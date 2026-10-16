// route_track.h – follow a route as a track instead of point to point.
//
// The waypoints are joined into one line, the track. On each update the boat
// steers for a target (the "carrot") a fixed distance along the track beyond
// the point on it closest to the boat. The boat converges onto the line and
// follows it through the waypoints without needing to hit any of them, and it
// starts each turn before the corner.
//
// Progress along the track only moves forward, so a waypoint the boat misses
// is left behind rather than circled back for. A route can be started from
// anywhere: the boat joins the track at its closest point.
//
// No Arduino dependencies, so it can be compiled and simulated on a computer.

#pragma once
#include <math.h>
#include <stdint.h>
#include "mesh_protocol.h"

// Where separate parts of the track pass about equally close to the boat (an
// out-and-back, a loop that ends where it began, a figure eight), the route
// is joined at the earliest part within this distance of the closest.
static constexpr float ROUTE_JOIN_TIE_M = 10.0f;

// Metres per degree of latitude (mean Earth radius).
static constexpr double ROUTE_M_PER_DEG_LAT = 111194.93;

struct RouteTrack {
    // Waypoints in a flat local frame: metres east (x) and north (y) of the
    // first waypoint. Over a few km this is far more accurate than the GPS.
    float   x[ROUTE_MAX_WAYPOINTS_PER_ROUTE];
    float   y[ROUTE_MAX_WAYPOINTS_PER_ROUTE];
    float   along[ROUTE_MAX_WAYPOINTS_PER_ROUTE]; // track distance from the start to each waypoint
    uint8_t count;
    double  originLat, originLon;
    double  mPerDegLon;
    float   progress;  // track distance to the boat's place on the track; never decreases
};

struct RouteGuidance {
    float   targetHeading;  // degrees true [0, 360), towards the carrot
    float   crossTrackM;    // distance from the boat's place on the track, positive = right of it
    float   progressM;      // track distance covered
    float   lengthM;        // track length
    uint8_t nextIndex;      // next waypoint ahead along the track
    float   nextDistanceM;  // straight-line distance to it
    bool    finished;       // inside the last waypoint's radius, or past the end of the track
};

inline void routeTrackToLocal(const RouteTrack& t, double lat, double lon, float& x, float& y) {
    double dLon = lon - t.originLon;
    if (dLon > 180.0)  dLon -= 360.0;
    if (dLon < -180.0) dLon += 360.0;
    x = (float)(dLon * t.mPerDegLon);
    y = (float)((lat - t.originLat) * ROUTE_M_PER_DEG_LAT);
}

inline void routeTrackBuild(RouteTrack& t, const WaypointData* wps, uint8_t count) {
    t.count = count;
    t.progress = 0.0f;
    if (count == 0) return;
    t.originLat = wps[0].latE7 / 1e7;
    t.originLon = wps[0].lonE7 / 1e7;
    t.mPerDegLon = ROUTE_M_PER_DEG_LAT * cos(t.originLat * M_PI / 180.0);
    for (uint8_t i = 0; i < count; ++i) {
        routeTrackToLocal(t, wps[i].latE7 / 1e7, wps[i].lonE7 / 1e7, t.x[i], t.y[i]);
        t.along[i] = (i == 0) ? 0.0f
                   : t.along[i - 1] + hypotf(t.x[i] - t.x[i - 1], t.y[i] - t.y[i - 1]);
    }
}

// Segment k runs from waypoint k to waypoint k + 1.

// The segment that track distance s lies on (the later one at a waypoint).
inline uint8_t routeTrackSegmentAt(const RouteTrack& t, float s) {
    uint8_t k = 0;
    while (k + 2 < t.count && t.along[k + 1] <= s) ++k;
    return k;
}

// Point at track distance s, which lies on segment k.
inline void routeTrackPoint(const RouteTrack& t, uint8_t k, float s, float& px, float& py) {
    float len = t.along[k + 1] - t.along[k];
    float f = (len > 0.0f) ? (s - t.along[k]) / len : 0.0f;
    px = t.x[k] + f * (t.x[k + 1] - t.x[k]);
    py = t.y[k] + f * (t.y[k + 1] - t.y[k]);
}

inline float routeTrackDistance(const RouteTrack& t, uint8_t k, float s, float bx, float by) {
    float px, py;
    routeTrackPoint(t, k, s, px, py);
    return hypotf(bx - px, by - py);
}

// Track distance of the point on segment k, between track distances lo and
// hi, that is closest to (bx, by).
inline float routeTrackClosest(const RouteTrack& t, uint8_t k, float bx, float by, float lo, float hi) {
    float len = t.along[k + 1] - t.along[k];
    float s = t.along[k];
    if (len > 0.0f) {
        s += ((bx - t.x[k]) * (t.x[k + 1] - t.x[k]) + (by - t.y[k]) * (t.y[k + 1] - t.y[k])) / len;
    }
    if (s < lo) s = lo;
    if (s > hi) s = hi;
    return s;
}

// Walking on from track distance s (on segment k, which is updated), the
// first point at least r from (bx, by), or the end of the track.
inline float routeTrackLeaveCircle(const RouteTrack& t, uint8_t& k, float s, float bx, float by, float r) {
    for (;;) {
        float len = t.along[k + 1] - t.along[k];
        if (len > 0.0f) {
            // Where the segment's line leaves the circle: |o + u·d| = r.
            float ux = (t.x[k + 1] - t.x[k]) / len, uy = (t.y[k + 1] - t.y[k]) / len;
            float ox = t.x[k] - bx, oy = t.y[k] - by;
            float b = ux * ox + uy * oy;
            float disc = b * b - (ox * ox + oy * oy - r * r);
            float leave = (disc > 0.0f) ? t.along[k] - b + sqrtf(disc) : s;
            if (leave < s) leave = s;
            if (leave <= t.along[k + 1]) return leave;
        }
        if (k + 2 >= t.count) return t.along[k + 1];
        s = t.along[++k];
    }
}

// Walking back from track distance s (on segment k) while the track keeps
// getting closer to (bx, by), where it stops — but not before track distance
// floor.
inline float routeTrackClosestBehind(const RouteTrack& t, uint8_t k, float s, float bx, float by, float floor) {
    for (;;) {
        float best = routeTrackClosest(t, k, bx, by, fmaxf(t.along[k], floor), s);
        if (best > t.along[k] || k == 0 || t.along[k] <= floor) return best;
        s = t.along[k];
        --k;
    }
}

// Join the track at the point closest to the boat. Where separate parts of
// it are about equally close, join the earliest.
inline void routeTrackJoin(RouteTrack& t, double lat, double lon) {
    t.progress = 0.0f;
    if (t.count < 2) return;
    float bx, by;
    routeTrackToLocal(t, lat, lon, bx, by);

    float nearest = INFINITY;
    for (uint8_t k = 0; k + 1 < t.count; ++k) {
        float s = routeTrackClosest(t, k, bx, by, t.along[k], t.along[k + 1]);
        nearest = fminf(nearest, routeTrackDistance(t, k, s, bx, by));
    }
    for (uint8_t k = 0; k + 1 < t.count; ++k) {
        float s = routeTrackClosest(t, k, bx, by, t.along[k], t.along[k + 1]);
        if (routeTrackDistance(t, k, s, bx, by) <= nearest + ROUTE_JOIN_TIE_M) {
            t.progress = s;
            return;
        }
    }
}

// Move the boat's place on the track forward and work out where to steer.
// lookaheadM is how far along the track to aim beyond that place;
// finishRadiusM is the last waypoint's arrival radius.
inline RouteGuidance routeTrackUpdate(RouteTrack& t, double lat, double lon,
                                      float lookaheadM, float finishRadiusM) {
    RouteGuidance g = {};
    if (t.count == 0) {
        g.finished = true;
        return g;
    }

    float bx, by;
    routeTrackToLocal(t, lat, lon, bx, by);
    const uint8_t last = t.count - 1;
    const float length = t.along[last];
    float cx = t.x[last], cy = t.y[last];  // the carrot
    bool carrotAtEnd = true;
    g.crossTrackM = hypotf(bx - t.x[0], by - t.y[0]);

    if (t.count >= 2) {
        // Follow the track forward while it keeps getting closer to the boat;
        // where it stops is the boat's place on it. A later part of the route
        // that happens to pass nearby is never jumped to, because the track
        // gets further away before it comes back.
        uint8_t k = routeTrackSegmentAt(t, t.progress);
        float s = routeTrackClosest(t, k, bx, by, t.progress, t.along[k + 1]);
        while (s >= t.along[k + 1] && k + 2 < t.count) {
            ++k;
            s = routeTrackClosest(t, k, bx, by, s, t.along[k + 1]);
        }
        t.progress = s;

        float c = fminf(s + lookaheadM, length);
        uint8_t ck = routeTrackSegmentAt(t, c);

        // Where the track turns back on itself (sharper than about 120°),
        // the carrot can land next to the boat or behind it, and the boat
        // would circle it. Push it on until it is at least half the lookahead
        // away, and put the boat on the part of the track that comes back.
        float minCarrot = lookaheadM * 0.5f;
        if (c < length && routeTrackDistance(t, ck, c, bx, by) < minCarrot) {
            c = routeTrackLeaveCircle(t, ck, c, bx, by, minCarrot);
            t.progress = fmaxf(t.progress, routeTrackClosestBehind(t, ck, c, bx, by, t.progress));
            k = routeTrackSegmentAt(t, t.progress);
        }
        routeTrackPoint(t, ck, c, cx, cy);
        carrotAtEnd = (c >= length);

        float px, py;
        routeTrackPoint(t, k, t.progress, px, py);
        g.crossTrackM = hypotf(bx - px, by - py);
        float side = (t.x[k + 1] - t.x[k]) * (by - py) - (t.y[k + 1] - t.y[k]) * (bx - px);
        if (side > 0.0f) g.crossTrackM = -g.crossTrackM;  // left of the track
    }

    g.nextIndex = 0;
    while (g.nextIndex < last && t.along[g.nextIndex] <= t.progress) ++g.nextIndex;
    g.nextDistanceM = hypotf(bx - t.x[g.nextIndex], by - t.y[g.nextIndex]);

    float heading = atan2f(cx - bx, cy - by) * (float)(180.0 / M_PI);
    g.targetHeading = (heading < 0.0f) ? heading + 360.0f : heading;
    g.progressM = t.progress;
    g.lengthM = length;

    // Only check the finish radius once aiming at the last waypoint, so a
    // loop that ends where it starts isn't finished straight away.
    float toEnd = hypotf(bx - t.x[last], by - t.y[last]);
    g.finished = (length > 0.0f && t.progress >= length) ||
                 (carrotAtEnd && toEnd <= finishRadiusM);
    return g;
}
