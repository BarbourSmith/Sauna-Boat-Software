// mesh_remote.h – logs, serial commands and firmware updates over the mesh,
// for modules whose USB port is hard to reach (steering and navigation).
//
// The handheld relays between its USB port and the mesh, driven from a
// computer by tools/mesh_gateway. A module using this header:
//   - prints through `Log` instead of `Serial`. Output still goes to the
//     serial port, and while a computer is listening each line is also
//     broadcast as MSG_REMOTE_LOG.
//   - runs text sent with MSG_REMOTE_COMMAND as if typed into its serial port.
//   - accepts new firmware (MSG_OTA_*). It is written to the spare app
//     partition and checked against its MD5 before the module restarts into
//     it. A module that is mid-transfer ignores all other mesh messages and
//     skips its normal loop, so whatever it was driving holds still.
//   - keeps new firmware only once it has heard the mesh. If it hears nothing
//     for REMOTE_CONFIRM_TIMEOUT_MS after the update, or crashes before then,
//     the bootloader goes back to the previous firmware.
//
// Include it once, from main.cpp, and:
//   setup():          meshRemoteBegin(MODULE_X, canUpdate, onCommand);  after esp_now_init()
//   receive callback: if (meshRemoteReceive(data, len)) return;         first thing
//   loop():           if (meshRemoteLoop()) return;                     first thing

#pragma once
#include <Arduino.h>
#include <esp_now.h>
#include <esp_ota_ops.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <Update.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include "mesh_protocol.h"

static constexpr unsigned long REMOTE_LISTEN_MS          = 3000;   // send logs this long after the last ping
static constexpr unsigned long REMOTE_OTA_IDLE_MS        = 10000;  // give up on an update after this long without a packet
static constexpr unsigned long REMOTE_CONFIRM_TIMEOUT_MS = 120000; // roll back new firmware that hears nothing this long
static constexpr int           REMOTE_LOG_QUEUE_LEN      = 16;     // lines waiting to be broadcast
static constexpr int           REMOTE_LOGS_PER_LOOP      = 4;      // lines broadcast per meshRemoteLoop()

struct RemoteLogLine {
    uint8_t seq;
    uint8_t len;
    char    text[REMOTE_LOG_TEXT_MAX];
};

// A mesh packet handed from the receive callback to loop().
struct RemotePacket {
    uint8_t len;
    uint8_t data[ESP_NOW_MAX_DATA_LEN];
};

static uint8_t            g_remoteModule     = 0;
static bool             (*g_remoteCanUpdate)() = nullptr;
static void             (*g_remoteOnCommand)(const char* text) = nullptr;
static QueueHandle_t      g_remoteLogQueue   = nullptr;
static QueueHandle_t      g_remoteRxQueue    = nullptr;  // OTA and command packets for loop()
static SemaphoreHandle_t  g_remoteLogMutex   = nullptr;

static volatile unsigned long g_remoteLastPingMs  = 0;
static volatile bool          g_remotePingPending = false;  // answer with MSG_REMOTE_INFO from loop()
static volatile bool          g_remoteHeardMesh   = false;
static volatile bool          g_remoteUpdating    = false;
static bool                   g_remoteUnconfirmed = false;  // booted from an update that isn't kept yet
static uint8_t                g_remoteSession     = 0;
static uint32_t               g_remoteReceived    = 0;  // image bytes accepted so far
static unsigned long          g_remoteLastOtaMs   = 0;

// The log line being built, guarded by g_remoteLogMutex.
static char    g_remoteLine[REMOTE_LOG_TEXT_MAX];
static size_t  g_remoteLineLen = 0;
static uint8_t g_remoteLogSeq  = 0;

static bool remoteListening() {
    unsigned long last = g_remoteLastPingMs;
    return last != 0 && millis() - last < REMOTE_LISTEN_MS;
}

static void remoteQueueLine() {
    if (g_remoteLineLen == 0) return;
    RemoteLogLine line;
    line.seq = g_remoteLogSeq++;
    line.len = (uint8_t)g_remoteLineLen;
    memcpy(line.text, g_remoteLine, g_remoteLineLen);
    g_remoteLineLen = 0;
    xQueueSend(g_remoteLogQueue, &line, 0);  // full: dropped, which shows as a gap in seq
}

// Print that writes to Serial and, while a computer is listening, the mesh.
// Safe to use from the ESP-NOW receive callback as well as loop().
class RemoteLog : public Print {
public:
    size_t write(uint8_t c) override { return write(&c, 1); }

    size_t write(const uint8_t* buf, size_t n) override {
        Serial.write(buf, n);
        if (!g_remoteLogMutex || !remoteListening()) return n;
        xSemaphoreTake(g_remoteLogMutex, portMAX_DELAY);
        for (size_t i = 0; i < n; ++i) {
            char ch = (char)buf[i];
            if (ch == '\r') continue;
            if (ch != '\n') g_remoteLine[g_remoteLineLen++] = ch;
            if (ch == '\n' || g_remoteLineLen == sizeof(g_remoteLine)) remoteQueueLine();
        }
        xSemaphoreGive(g_remoteLogMutex);
        return n;
    }
};

static RemoteLog Log;

// Arduino confirms freshly updated firmware as soon as it starts unless this
// returns true. meshRemoteLoop() confirms it once it has heard the mesh.
extern "C" bool verifyRollbackLater() { return true; }

static void remoteSend(const void* msg, size_t len) {
    esp_now_send(MESH_BROADCAST_ADDR, reinterpret_cast<const uint8_t*>(msg), len);
}

static void remoteSendInfo() {
    RemoteInfoMessage m = {};
    m.type        = MSG_REMOTE_INFO;
    m.src         = g_remoteModule;
    m.resetReason = (uint8_t)esp_reset_reason();
    m.otaState    = g_remoteUpdating ? REMOTE_FW_UPDATING
                  : g_remoteUnconfirmed ? REMOTE_FW_UNCONFIRMED : REMOTE_FW_CONFIRMED;
    m.uptimeS     = millis() / 1000;
    strncpy(m.build, __DATE__ " " __TIME__, sizeof(m.build) - 1);
    remoteSend(&m, sizeof(m));
}

static void remoteSendLogs() {
    RemoteLogLine line;
    for (int i = 0; i < REMOTE_LOGS_PER_LOOP &&
                    xQueueReceive(g_remoteLogQueue, &line, 0) == pdTRUE; ++i) {
        RemoteLogMessage m;
        m.type = MSG_REMOTE_LOG;
        m.src  = g_remoteModule;
        m.seq  = line.seq;
        memcpy(m.text, line.text, line.len);
        remoteSend(&m, offsetof(RemoteLogMessage, text) + line.len);
    }
}

static void remoteAck(uint8_t session, uint8_t status) {
    OtaAckMessage m;
    m.type       = MSG_OTA_ACK;
    m.src        = g_remoteModule;
    m.session    = session;
    m.status     = status;
    // Not Update.progress(): that only counts bytes once a whole flash sector
    // has been written.
    m.nextOffset = g_remoteUpdating ? g_remoteReceived : 0;
    remoteSend(&m, sizeof(m));
}

static void remoteStopUpdate(const char* why) {
    Update.abort();
    g_remoteUpdating = false;
    Log.printf("[Remote] Update stopped: %s\n", why);
}

static void remoteHandleBegin(const RemotePacket& p) {
    if (p.len < sizeof(OtaBeginMessage)) return;
    OtaBeginMessage m;
    memcpy(&m, p.data, sizeof(m));

    if (g_remoteUpdating) {
        if (m.session == g_remoteSession) {  // our earlier ack was lost
            remoteAck(m.session, OTA_STATUS_OK);
            return;
        }
        remoteStopUpdate("a new update started");
    }
    if (g_remoteCanUpdate && !g_remoteCanUpdate()) {
        Log.println("[Remote] Update refused: the module is steering the boat.");
        remoteAck(m.session, OTA_STATUS_BUSY);
        return;
    }
    if (!Update.begin(m.size)) {
        Log.printf("[Remote] Update refused: %s\n", Update.errorString());
        remoteAck(m.session, Update.getError() == UPDATE_ERROR_SIZE ? OTA_STATUS_TOO_BIG
                                                                     : OTA_STATUS_FLASH);
        return;
    }
    char md5[33];
    for (int i = 0; i < 16; ++i) snprintf(md5 + 2 * i, 3, "%02x", m.md5[i]);
    Update.setMD5(md5);

    g_remoteUpdating  = true;
    g_remoteSession   = m.session;
    g_remoteReceived  = 0;
    g_remoteLastOtaMs = millis();
    Log.printf("[Remote] Receiving new firmware (%lu bytes).\n", (unsigned long)m.size);
    remoteAck(m.session, OTA_STATUS_OK);
}

static void remoteHandleData(const RemotePacket& p) {
    const size_t header = offsetof(OtaDataMessage, data);
    if (p.len <= header || p.len > sizeof(OtaDataMessage)) return;
    OtaDataMessage m;
    memcpy(&m, p.data, p.len);
    if (!g_remoteUpdating || m.session != g_remoteSession) {
        remoteAck(m.session, OTA_STATUS_NO_SESSION);
        return;
    }
    g_remoteLastOtaMs = millis();

    // Only the chunk that continues the image is written; for anything else
    // the ack's nextOffset tells the handheld where to carry on from.
    size_t n = p.len - header;
    if (m.offset == g_remoteReceived) {
        if (Update.write(m.data, n) != n) {
            Log.printf("[Remote] Flash write failed: %s\n", Update.errorString());
            remoteAck(m.session, OTA_STATUS_FLASH);
            remoteStopUpdate("flash write failed");
            return;
        }
        g_remoteReceived += n;
    }
    remoteAck(m.session, OTA_STATUS_OK);
}

static void remoteHandleEnd(const RemotePacket& p) {
    if (p.len < sizeof(OtaControlMessage)) return;
    OtaControlMessage m;
    memcpy(&m, p.data, sizeof(m));
    if (!g_remoteUpdating || m.session != g_remoteSession) {
        remoteAck(m.session, OTA_STATUS_NO_SESSION);
        return;
    }
    // Checks the length and MD5, and that the image is a valid app for this
    // chip, before making it the one to boot.
    if (!Update.end()) {
        Log.printf("[Remote] New firmware rejected: %s\n", Update.errorString());
        remoteAck(m.session, OTA_STATUS_VERIFY);
        g_remoteUpdating = false;
        return;
    }
    remoteAck(m.session, OTA_STATUS_OK);
    Log.println("[Remote] New firmware verified; restarting into it.");
    remoteSendLogs();
    delay(200);  // let the ack and log go out
    ESP.restart();
}

static void remoteHandleAbort(const RemotePacket& p) {
    if (p.len < sizeof(OtaControlMessage)) return;
    OtaControlMessage m;
    memcpy(&m, p.data, sizeof(m));
    if (g_remoteUpdating && m.session == g_remoteSession) {
        remoteStopUpdate("cancelled from the computer");
    }
    remoteAck(m.session, OTA_STATUS_OK);
}

static void remoteHandleCommand(const RemotePacket& p) {
    const size_t header = offsetof(RemoteCommandMessage, text);
    if (p.len <= header || !g_remoteOnCommand) return;
    char text[REMOTE_COMMAND_TEXT_MAX + 1];
    size_t n = min((size_t)(p.len - header), (size_t)REMOTE_COMMAND_TEXT_MAX);
    memcpy(text, p.data + header, n);
    text[n] = '\0';
    Log.printf("[Remote] Command: %s\n", text);
    g_remoteOnCommand(text);
}

// canUpdate (optional) returns false while an update would be unsafe, e.g.
// while the module is steering. onCommand (optional) runs remote commands.
static void meshRemoteBegin(uint8_t module, bool (*canUpdate)() = nullptr,
                            void (*onCommand)(const char* text) = nullptr) {
    g_remoteModule    = module;
    g_remoteCanUpdate = canUpdate;
    g_remoteOnCommand = onCommand;
    g_remoteLogQueue  = xQueueCreate(REMOTE_LOG_QUEUE_LEN, sizeof(RemoteLogLine));
    g_remoteRxQueue   = xQueueCreate(4, sizeof(RemotePacket));
    g_remoteLogMutex  = xSemaphoreCreateMutex();

    if (!esp_now_is_peer_exist(MESH_BROADCAST_ADDR)) {
        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, MESH_BROADCAST_ADDR, 6);
        peer.channel = MESH_WIFI_CHANNEL;
        peer.ifidx   = WIFI_IF_STA;
        peer.encrypt = false;
        esp_now_add_peer(&peer);
    }

    esp_ota_img_states_t state;
    g_remoteUnconfirmed =
        esp_ota_get_state_partition(esp_ota_get_running_partition(), &state) == ESP_OK &&
        state == ESP_OTA_IMG_PENDING_VERIFY;
    if (g_remoteUnconfirmed) {
        Log.println("[Remote] Running new firmware; it is kept once the mesh is heard.");
    }
    remoteSendInfo();
}

// Call first in the ESP-NOW receive callback. Returns true when the module
// should ignore the packet: it was for this header, or an update is running.
static bool meshRemoteReceive(const uint8_t* data, int len) {
    if (len < 2) return g_remoteUpdating;
    g_remoteHeardMesh = true;
    uint8_t type = data[0];

    if (type == MSG_REMOTE_PING) {
        g_remoteLastPingMs  = millis();
        g_remotePingPending = true;
        return true;
    }
    if (type == MSG_REMOTE_COMMAND || (type >= MSG_OTA_BEGIN && type <= MSG_OTA_ABORT)) {
        if (len >= 3 && data[1] == MODULE_HANDHELD && data[2] == g_remoteModule &&
            len <= ESP_NOW_MAX_DATA_LEN && g_remoteRxQueue) {
            RemotePacket p;
            p.len = (uint8_t)len;
            memcpy(p.data, data, len);
            xQueueSend(g_remoteRxQueue, &p, 0);
        }
        return true;
    }
    if (type == MSG_REMOTE_INFO || type == MSG_REMOTE_LOG || type == MSG_OTA_ACK) {
        return true;  // another module talking to the handheld
    }
    return g_remoteUpdating;
}

// Call first in loop(). Returns true while an update is being received; the
// module should then skip the rest of its loop.
static bool meshRemoteLoop() {
    if (!g_remoteRxQueue) return false;  // meshRemoteBegin() not called (ESP-NOW failed)
    unsigned long now = millis();

    if (g_remoteUnconfirmed) {
        if (g_remoteHeardMesh) {
            esp_ota_mark_app_valid_cancel_rollback();
            g_remoteUnconfirmed = false;
            Log.println("[Remote] New firmware kept: the mesh is working.");
        } else if (now >= REMOTE_CONFIRM_TIMEOUT_MS) {
            Serial.println("[Remote] Nothing heard on the mesh; going back to the previous firmware.");
            esp_ota_mark_app_invalid_rollback_and_reboot();
        }
    }

    if (g_remotePingPending) {
        g_remotePingPending = false;
        remoteSendInfo();
    }
    remoteSendLogs();

    // While updating, wait briefly for the next chunk instead of spinning.
    RemotePacket p;
    TickType_t wait = g_remoteUpdating ? pdMS_TO_TICKS(5) : 0;
    while (xQueueReceive(g_remoteRxQueue, &p, wait) == pdTRUE) {
        wait = 0;
        switch (p.data[0]) {
            case MSG_OTA_BEGIN:      remoteHandleBegin(p);   break;
            case MSG_OTA_DATA:       remoteHandleData(p);    break;
            case MSG_OTA_END:        remoteHandleEnd(p);     break;
            case MSG_OTA_ABORT:      remoteHandleAbort(p);   break;
            case MSG_REMOTE_COMMAND: remoteHandleCommand(p); break;
        }
    }

    if (g_remoteUpdating && millis() - g_remoteLastOtaMs > REMOTE_OTA_IDLE_MS) {
        remoteStopUpdate("nothing received for 10 s");
    }
    return g_remoteUpdating;
}
