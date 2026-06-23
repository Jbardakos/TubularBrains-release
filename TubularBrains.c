/* TubularBrains by tautologos — demo / early build, work in progress. Provided as-is. */
/*
 * TubularBrains — interactive multi-node tube tool for LightWave Modeler
 * -----------------------------------------------------------------
 * Class:  MeshEditTool (LWMESHEDITTOOL_CLASS) — the INTERACTIVE class.
 *
 * Interaction:
 *   DRAG a handle : move a node / scale handle / TWIST handle / branch tip —
 *                   works in EVERY viewport (handles exposed via count()/handle()
 *                   /adjust() so Modeler picks them natively, like capsule).
 *                   The twist handle spins each node's cross-section about the
 *                   tube axis — the per-node ORIENTATION control that removes
 *                   cross-section twist/warp ("rubber ducky") on bends.
 *   LEFT empty    : click-drag to start; click empty to EXTEND the chain (the
 *                   new node KEEPS the last node's thickness); click ON a
 *                   segment to SPLIT it.
 *   TAB           : subdivide the chain (insert a midpoint node between every
 *                   pair) and keep all handles live — blocks Modeler's Tab.
 *
 * Numeric panel: Thickness, Sides, Caps (None/Flat/Rounded), Tube Weight Map
 *   (a 0..1 VMap along the chain), Symmetry (X), Scale Handles, Twist Handles,
 *   Make Curve, Make 2pt Poly, Clear.
 *
 * Committing: the TUBE is a live preview; it BAKES when you DROP the tool (pick
 *   another tool / press the drop key). "Clear" discards.
 *   - "Make Curve" / "Make 2pt Poly" are TOGGLES. When on, the node centreline is
 *     ALSO emitted on the NEXT (core) layer — as a CURVE and/or a 2-point-poly
 *     chain — as part of the SAME build as the tube, so it bakes together with
 *     the tube on drop. (This is the mechanism that worked in the early builds;
 *     a separate one-shot button commit did not.)
 *
 * Output geometry: a swept tube through all nodes. Consecutive rings SHARE
 *   their points, so the chain bakes as ONE continuous, already-merged mesh.
 *   Interior rings are miter-compensated for uniform wall thickness through bends.
 */

#include <lwserver.h>
#include <lwmeshedt.h>     /* MeshEditOp: addPoint/addFace/addCurve/setLayer */
#include <lwtool.h>        /* LWToolFuncs, LWToolEvent, LWWireDrawAccess     */
#include <lwmodtool.h>     /* LWMeshEditTool, LWMESHEDITTOOL_CLASS/_VERSION  */
#include <lwxpanel.h>      /* numeric panel                                  */

#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Dual-key code (TAB) we react to in the tool's key() callback. */
#define KEY_TAB    ((LWDualKey)'\t')

/* Control / value IDs — XPanels reserves everything below 0x8000. */
#define ID_THICK   0x8001
#define ID_SIDES   0x8002
#define ID_CAPS    0x8003
#define ID_SHOWH   0x8005
#define ID_APPLY   0x8006
#define ID_CLEAR   0x8007
#define ID_BRANCH  0x8008          /* "Branch Mode" toggle                   */
#define ID_BSEGS   0x8009          /* branch length segments                 */
#define ID_TWIST   0x800A          /* "Twist Handles" toggle                 */
#define ID_WMAP    0x800B          /* "Tube Weight Map" toggle (0->1)        */
#define ID_SYM     0x800D          /* "Symmetry (X)" toggle                  */
#define ID_BAKEC   0x800E          /* "Make Curve" spine button              */
#define ID_BAKEP   0x800F          /* "Make 2pt Poly" spine button           */

#define SURFACE_NAME "TubularBrains"
#define WMAP_NAME    "TubularBrains"

#define SPINE_LAYER_OFFSET 1       /* curve/2pt-poly spine -> base+1 (core)  */

#define MAX_NODES    128
#define MINR       1.0e-4          /* smallest allowed node radius (0.1 mm)  */

/* Cap styles */
#define CAP_NONE    0
#define CAP_FLAT    1
#define CAP_ROUND   2

/* ---- instance data ---------------------------------------------- */
typedef struct st_CurveNode {
    double pos[3];         /* node position (model space)             */
    double radius;         /* per-node tube radius (the scale handle) */
    double roll;           /* per-node twist about the tube axis (rad)*/
} CurveNode;

typedef struct st_CurveData {
    CurveNode node[MAX_NODES];
    int    nnodes;         /* number of nodes in the chain            */
    int    drag;           /* handle index being raw-dragged, -1 none */
    double thickness;      /* default radius applied to NEW nodes     */
    int    sides;          /* radial segments == "interpolation"      */
    int    caps;           /* CAP_*                                   */
    int    showHandles;    /* draw/pick per-node scale handles?       */
    int    showTwist;      /* draw/pick per-node twist (roll) handles?*/
    int    wmap;           /* emit a 0..1 tube weight map?             */
    int    symmetry;       /* mirror everything across X=0?            */
    int    mkCurve;        /* also emit a CURVE spine on next layer?   */
    int    mkPoly;         /* also emit a 2pt-poly spine on next layer?*/
    int    baking;         /* set on bake so End() resets once        */
    int    dirty;          /* needs redraw?                           */
    int    update;         /* LWT_TEST_* code returned by Test()      */
} CurveData;

/* XPanel function table, grabbed once at activation. */
static LWXPanelFuncs *xpanf = NULL;

/* ---- small vector helpers --------------------------------------- */
static void v_copy(const double *u, double *o) { o[0]=u[0]; o[1]=u[1]; o[2]=u[2]; }
static void v_sub(const double *u, const double *v, double *o) {
    o[0] = (u[0] - v[0]); o[1] = (u[1] - v[1]); o[2] = (u[2] - v[2]);
}
static void v_add(const double *u, const double *v, double *o) {
    o[0] = (u[0] + v[0]); o[1] = (u[1] + v[1]); o[2] = (u[2] + v[2]);
}
static double v_dot(const double *u, const double *v) {
    return ((u[0]*v[0]) + (u[1]*v[1])) + (u[2]*v[2]);
}
static double v_len(const double *u) { return sqrt(v_dot(u, u)); }
static double v_dist(const double *u, const double *v) {
    double d[3]; v_sub(u, v, d); return v_len(d);
}
static void v_cross(const double *u, const double *v, double *o) {
    o[0] = ((u[1] * v[2]) - (u[2] * v[1]));
    o[1] = ((u[2] * v[0]) - (u[0] * v[2]));
    o[2] = ((u[0] * v[1]) - (u[1] * v[0]));
}
static void v_norm(double *u) {
    double l = v_len(u);
    if (l > 1e-12) { u[0] /= l; u[1] /= l; u[2] /= l; }
}
static void v_madd(const double *p, const double *d, double s, double *o) {
    o[0] = p[0] + (d[0]*s); o[1] = p[1] + (d[1]*s); o[2] = p[2] + (d[2]*s);
}

/* A safe unit copy of the viewport depth axis (LWToolEvent.axis). Points
   under the mouse all lie along this axis, so its component is the part of
   a position we CANNOT see/control in the current view. */
static void safe_axis(const double *a, double *o) {
    v_copy(a, o);
    if (v_len(o) < 1e-9) { o[0]=0.0; o[1]=0.0; o[2]=1.0; } else v_norm(o);
}
/* Project v into the view plane (drop its component along unit axis n). */
static void v_projperp(const double *v, const double *n, double *o) {
    double d = v_dot(v, n);
    o[0] = v[0] - d*n[0]; o[1] = v[1] - d*n[1]; o[2] = v[2] - d*n[2];
}
/* Distance between two points AS SEEN in the current view (ignores depth),
   so handle picking works the same in every viewport. */
static double pick_dist(const double *p, const double *q, const double *axis) {
    double d[3];
    v_sub(p, q, d); v_projperp(d, axis, d);
    return v_len(d);
}

/* Move a node to the cursor IN THE VIEW PLANE while keeping the depth it
   already has along the view axis. Uses absolute positions (no accumulated
   deltas), so it is independent of window size and never drifts:
       node <- cursor + (node.axis - cursor.axis) * axis
   In-plane the node sits exactly under the cursor; along the (invisible)
   view axis it keeps the coordinate it was given in other viewports. */
static void node_to_cursor(double *node, const double *cursor, const double *axis) {
    double dn = v_dot(node, axis), dc = v_dot(cursor, axis), s = dn - dc;
    node[0] = cursor[0] + s*axis[0];
    node[1] = cursor[1] + s*axis[1];
    node[2] = cursor[2] + s*axis[2];
}

/* Rotate vector v around unit axis k by angle (Rodrigues' formula). */
static void v_rotate(const double *v, const double *k, double ang, double *out) {
    double c = cos(ang), s = sin(ang);
    double kv[3], kd = v_dot(k, v);
    v_cross(k, v, kv);
    out[0] = ((v[0]*c) + (kv[0]*s)) + (k[0]*kd*(1.0-c));
    out[1] = ((v[1]*c) + (kv[1]*s)) + (k[1]*kd*(1.0-c));
    out[2] = ((v[2]*c) + (kv[2]*s)) + (k[2]*kd*(1.0-c));
}

/* ---- chain frames ----------------------------------------------- */

/* Tangent at node i: averaged (mitered) for interior nodes, single-sided
   at the ends. */
static void node_tangent(CurveData *cd, int i, double *T)
{
    int n = cd->nnodes;
    if (n < 2) { T[0] = 0.0; T[1] = 0.0; T[2] = 1.0; return; }
    if (i == 0) {
        v_sub(cd->node[1].pos, cd->node[0].pos, T);
    } else if (i == n - 1) {
        v_sub(cd->node[n-1].pos, cd->node[n-2].pos, T);
    } else {
        double a[3], b[3];
        v_sub(cd->node[i].pos,   cd->node[i-1].pos, a); v_norm(a);
        v_sub(cd->node[i+1].pos, cd->node[i].pos,   b); v_norm(b);
        v_add(a, b, T);
    }
    v_norm(T);
    if (v_len(T) < 0.5) { T[0] = 0.0; T[1] = 0.0; T[2] = 1.0; } /* degenerate */
}

/* Per-node orthonormal ring frame (R = right, U = up), built by
   parallel-transport along the chain so the tube doesn't twist. */
static void compute_frames(CurveData *cd, double R[][3], double U[][3])
{
    int n = cd->nnodes, i;
    double T[3], Tprev[3], up[3];

    if (n < 1) return;

    node_tangent(cd, 0, T);
    up[0] = 0.0; up[1] = 1.0; up[2] = 0.0;
    if (fabs(T[1]) > 0.99) { up[0] = 1.0; up[1] = 0.0; up[2] = 0.0; }
    v_cross(T, up, R[0]); v_norm(R[0]);
    v_cross(T, R[0], U[0]); v_norm(U[0]);
    v_copy(T, Tprev);

    for (i = 1; i < n; i++) {
        double axis[3], s, c, ang, d;
        node_tangent(cd, i, T);
        v_cross(Tprev, T, axis);
        s = v_len(axis);
        c = v_dot(Tprev, T);
        if (s < 1e-9) {
            v_copy(R[i-1], R[i]);            /* tangent unchanged */
        } else {
            v_norm(axis);
            ang = atan2(s, c);
            v_rotate(R[i-1], axis, ang, R[i]);
        }
        d = v_dot(R[i], T);                  /* re-orthonormalise */
        R[i][0] -= d*T[0]; R[i][1] -= d*T[1]; R[i][2] -= d*T[2];
        v_norm(R[i]);
        v_cross(T, R[i], U[i]); v_norm(U[i]);
        v_copy(T, Tprev);
    }
}

/* Per-node miter data so the wall thickness stays uniform through bends:
   along bend[i], ring coords are stretched by st[i] = 1/cos(half-angle). */
static void compute_miter(CurveData *cd, double *st, double bend[][3])
{
    int n = cd->nnodes, i;
    for (i = 0; i < n; i++) {
        st[i] = 1.0; bend[i][0] = bend[i][1] = bend[i][2] = 0.0;
        if (i == 0 || i == n - 1) continue;
        {
            double d1[3], d2[3], sum[3], diff[3], T[3], cf;
            v_sub(cd->node[i].pos,   cd->node[i-1].pos, d1); v_norm(d1);
            v_sub(cd->node[i+1].pos, cd->node[i].pos,   d2); v_norm(d2);
            v_add(d1, d2, sum);
            if (v_len(sum) < 1e-6) continue;          /* ~180 deg fold */
            v_copy(sum, T); v_norm(T);
            cf = v_dot(d2, T);
            if (cf < 0.25) cf = 0.25;                 /* clamp sharp bends */
            v_sub(d2, d1, diff);
            if (v_len(diff) < 1e-6) continue;         /* straight */
            v_norm(diff);
            st[i] = 1.0 / cf;
            v_copy(diff, bend[i]);
        }
    }
}

/* Unit ring offset at node i, angle th, miter-stretched. */
static void ring_offset(const double *R, const double *U, double th,
                        double stretch, const double *bend, double *o)
{
    double c = cos(th), s = sin(th), pr;
    o[0] = (R[0]*c) + (U[0]*s);
    o[1] = (R[1]*c) + (U[1]*s);
    o[2] = (R[2]*c) + (U[2]*s);
    if (stretch > 1.0) {
        pr = v_dot(o, bend);
        o[0] += (stretch-1.0)*pr*bend[0];
        o[1] += (stretch-1.0)*pr*bend[1];
        o[2] += (stretch-1.0)*pr*bend[2];
    }
}

/* Rotate an orthonormal ring frame (R,U) about its tangent by `roll` radians.
   This is the per-node orientation control that fixes cross-section twist
   ("rubber ducky") — it spins the profile around the tube axis. */
static void roll_frame(double *R, double *U, double roll)
{
    double c = cos(roll), s = sin(roll), Rn[3];
    Rn[0] = R[0]*c + U[0]*s; Rn[1] = R[1]*c + U[1]*s; Rn[2] = R[2]*c + U[2]*s;
    U[0] = U[0]*c - R[0]*s;  U[1] = U[1]*c - R[1]*s;  U[2] = U[2]*c - R[2]*s;
    R[0] = Rn[0]; R[1] = Rn[1]; R[2] = Rn[2];
}

/* Rolled ring frame (and tangent) for node k. */
static void node_frame(CurveData *cd, int k, double *R, double *U, double *T)
{
    double Ra[MAX_NODES][3], Ua[MAX_NODES][3];
    compute_frames(cd, Ra, Ua);
    node_tangent(cd, k, T);
    v_copy(Ra[k], R); v_copy(Ua[k], U);
    roll_frame(R, U, cd->node[k].roll);
}

/* ---- framework handle block layout ------------------------------
   [0,n)   node position handles
   [n,2n)  per-node scale handles   (only when Scale Handles on)
   [2n,3n) per-node twist handles   (only when Twist Handles on)         */
static int fw_count(CurveData *cd)
{
    return (cd->nnodes < 1) ? 0 : (cd->nnodes * 3);
}
/* Position + priority of framework handle i, or 0 if invalid/hidden. */
static int fw_pos(CurveData *cd, int i, double *pos)
{
    int n = cd->nnodes;
    if (i < 0) return 0;
    if (i < n) { v_copy(cd->node[i].pos, pos); return i+1; }
    if (i < 2*n) {                               /* scale: along rolled R */
        int k = i - n; double R[3], U[3], T[3], r = cd->node[k].radius;
        if (!cd->showHandles) return 0;
        if (r < MINR) r = MINR; node_frame(cd, k, R, U, T);
        v_madd(cd->node[k].pos, R, r, pos); return i+1;
    }
    if (i < 3*n) {                               /* twist: along rolled U */
        int k = i - 2*n; double R[3], U[3], T[3], r = cd->node[k].radius;
        if (!cd->showTwist) return 0;
        if (r < MINR) r = MINR; node_frame(cd, k, R, U, T);
        v_madd(cd->node[k].pos, U, r*1.5, pos); return i+1;
    }
    return 0;
}

/* Nearest segment to point p that p falls "in between", within tolerance.
   Returns the segment's first node index (>=0) and fills projOut, or -1. */
static int find_split(CurveData *cd, const double *p, const double *axis, double *projOut)
{
    int i, best = -1;
    double bestd = 0.0;
    for (i = 0; i < cd->nnodes - 1; i++) {
        const double *a = cd->node[i].pos, *b = cd->node[i+1].pos;
        double ab[3], ap[3], denom, t, proj[3], d, r, thr;
        v_sub(b, a, ab); v_sub(p, a, ap);
        denom = v_dot(ab, ab);
        if (denom < 1e-12) continue;
        t = v_dot(ap, ab) / denom;
        if (t < 0.1 || t > 0.9) continue;        /* must be "in between" */
        v_madd(a, ab, t, proj);
        d = pick_dist(p, proj, axis);            /* in-view distance */
        r = (cd->node[i].radius > cd->node[i+1].radius) ? cd->node[i].radius : cd->node[i+1].radius;
        if (r < cd->thickness) r = cd->thickness;
        thr = r * 2.5;
        if (d > thr) continue;
        if (best < 0 || d < bestd) { best = i; bestd = d; v_copy(proj, projOut); }
    }
    return best;
}

/* ---- symmetry: mirror-aware point/face emitters --------------------------
   When g_mir is set, every emitted point is reflected across X=0 and every
   polygon's winding is reversed (so the mirrored half keeps correct outward
   normals). All geometry goes through addPt/addFc, and Build() simply emits a
   second pass with g_mir=1 to produce a live, symmetric (X) copy. */
static int g_mir = 0;

static LWPntID addPt(MeshEditOp *op, const double *p)
{
    double q[3];
    q[0] = g_mir ? -p[0] : p[0]; q[1] = p[1]; q[2] = p[2];
    return op->addPoint(op->state, q);
}
static LWPolID addFc(MeshEditOp *op, const char *surf, int n, const LWPntID *v)
{
    LWPntID r[256]; int i;
    if (!g_mir) return op->addFace(op->state, surf, n, v);
    if (n > 256) n = 256;
    for (i = 0; i < n; i++) r[i] = v[n-1-i];     /* reverse winding for mirror */
    return op->addFace(op->state, surf, n, r);
}

/* Build a hemispherical dome cap from an existing seam ring outward. */
static void build_dome(MeshEditOp *op, const double *center, const double *Tout,
                       const double *R, const double *U, double r, int sides,
                       LWPntID *seam)
{
    int capSegs = sides / 4; if (capSegs < 2) capSegs = 2;
    LWPntID *prev = (LWPntID *)malloc(sizeof(LWPntID) * (size_t)sides);
    LWPntID *cur  = (LWPntID *)malloc(sizeof(LWPntID) * (size_t)sides);
    LWPntID pole;
    double p[3], fT[3];
    int lat, k, rev;

    if (!prev || !cur) { free(prev); free(cur); return; }
    memcpy(prev, seam, sizeof(LWPntID) * (size_t)sides);

    /* The seam ring is wound CCW about the frame tangent T = R x U. For the
       dome to face OUTWARD (along Tout), the quad winding must flip when Tout
       runs WITH T (the tip end) and stay as-is when it runs AGAINST T (the
       base end, where Tout = -T). Without this the tip cap is inside-out. */
    v_cross(R, U, fT);
    rev = (v_dot(fT, Tout) > 0.0);

    for (lat = 1; lat < capSegs; lat++) {
        double a  = (M_PI*0.5) * ((double)lat / (double)capSegs);
        double rr = r * cos(a), off = r * sin(a), c[3], o[3];
        v_madd(center, Tout, off, c);
        for (k = 0; k < sides; k++) {
            double th = (2.0*M_PI*(double)k) / (double)sides;
            ring_offset(R, U, th, 1.0, NULL, o);
            v_madd(c, o, rr, p);
            cur[k] = addPt(op, p);
        }
        for (k = 0; k < sides; k++) {
            int kn = (k + 1) % sides;
            LWPntID q[4];
            if (rev) { q[0]=prev[k]; q[1]=prev[kn]; q[2]=cur[kn]; q[3]=cur[k]; }
            else     { q[0]=prev[k]; q[1]=cur[k];  q[2]=cur[kn]; q[3]=prev[kn]; }
            addFc(op, SURFACE_NAME, 4, q);
        }
        memcpy(prev, cur, sizeof(LWPntID) * (size_t)sides);
    }

    v_madd(center, Tout, r, p);
    pole = addPt(op, p);
    for (k = 0; k < sides; k++) {
        int kn = (k + 1) % sides;
        LWPntID t[3];
        if (rev) { t[0]=prev[k]; t[1]=prev[kn]; t[2]=pole; }
        else     { t[0]=prev[k]; t[1]=pole;     t[2]=prev[kn]; }
        addFc(op, SURFACE_NAME, 3, t);
    }
    free(prev); free(cur);
}

/* ================================================================= */
/* Geometry generation (LIVE PREVIEW — never call op->done() here).   */
/* ================================================================= */
static LWError Build(void *inst, MeshEditOp *op)
{
    CurveData *cd = (CurveData *)inst;
    int      n     = cd->nnodes;
    int      sides = (cd->sides < 3) ? 3 : cd->sides;
    double   R[MAX_NODES][3], U[MAX_NODES][3];
    double   st[MAX_NODES], bend[MAX_NODES][3];
    double   wgt[MAX_NODES];
    LWPntID *ring  = NULL;
    int      i, k, pass;
    double   maxd  = 0.0;

    if (n < 2) return NULL;

    for (i = 0; i < n - 1; i++) {
        double dd = v_dist(cd->node[i].pos, cd->node[i+1].pos);
        if (dd > maxd) maxd = dd;
    }
    if (maxd < 1e-7) return NULL;                /* degenerate (just clicked) */

    compute_frames(cd, R, U);
    compute_miter(cd, st, bend);

    /* per-node orientation: spin each ring frame about its tangent by roll */
    for (i = 0; i < n; i++)
        if (cd->node[i].roll != 0.0) roll_frame(R[i], U[i], cd->node[i].roll);

    /* Per-node weight (0 at the first node -> 1 at the last) by arc length,
       for the optional Weight Map. (Mirror-independent — computed once.) */
    {
        double clen[MAX_NODES], total = 0.0;
        clen[0] = 0.0;
        for (i = 1; i < n; i++) { total += v_dist(cd->node[i].pos, cd->node[i-1].pos); clen[i] = total; }
        for (i = 0; i < n; i++)
            wgt[i] = (total > 1e-9) ? (clen[i] / total) : ((n > 1) ? (double)i/(double)(n-1) : 0.0);
    }

    /* Emit the geometry once; in Symmetry mode emit a second X-mirrored pass
       (g_mir reflects points across X=0 and reverses winding). */
    for (pass = 0; pass < (cd->symmetry ? 2 : 1); pass++) {
    g_mir = pass;

    ring = (LWPntID *)malloc(sizeof(LWPntID) * (size_t)n * (size_t)sides);
    if (!ring) { g_mir = 0; return "TubularBrains: out of memory"; }

    /* one ring of (miter-corrected) points per node */
    for (i = 0; i < n; i++) {
        double r = cd->node[i].radius; if (r < MINR) r = MINR;
        for (k = 0; k < sides; k++) {
            double th = (2.0 * M_PI * (double)k) / (double)sides, o[3], p[3];
            LWPntID pid;
            ring_offset(R[i], U[i], th, st[i], bend[i], o);
            v_madd(cd->node[i].pos, o, r, p);
            pid = addPt(op, p);
            ring[(i*sides) + k] = pid;
            if (cd->wmap && op->pntVMap) {        /* 0..1 gradient weight map */
                float wf = (float)wgt[i];
                op->pntVMap(op->state, pid, LWVMAP_WGHT, WMAP_NAME, 1, &wf);
            }
        }
    }

    /* side quads between consecutive (shared) rings -> merged tube */
    for (i = 0; i < n - 1; i++) {
        for (k = 0; k < sides; k++) {
            int kn = (k + 1) % sides;
            LWPntID quad[4];
            quad[0] = ring[(i*sides)     + k ];
            quad[1] = ring[(i*sides)     + kn];
            quad[2] = ring[((i+1)*sides) + kn];
            quad[3] = ring[((i+1)*sides) + k ];
            addFc(op, SURFACE_NAME, 4, quad);
        }
    }

    /* caps */
    if (cd->caps == CAP_FLAT) {
        LWPntID *cap = (LWPntID *)malloc(sizeof(LWPntID) * (size_t)sides);
        if (cap) {
            for (k = 0; k < sides; k++) cap[k] = ring[(sides - 1) - k];
            addFc(op, SURFACE_NAME, sides, cap);
            for (k = 0; k < sides; k++) cap[k] = ring[((n-1)*sides) + k];
            addFc(op, SURFACE_NAME, sides, cap);
            free(cap);
        }
    } else if (cd->caps == CAP_ROUND) {
        double T0[3], Tn[3], Tout[3], r0, rn;
        node_tangent(cd, 0, T0);
        node_tangent(cd, n-1, Tn);
        r0 = cd->node[0].radius;   if (r0 < MINR) r0 = MINR;
        rn = cd->node[n-1].radius; if (rn < MINR) rn = MINR;
        Tout[0] = -T0[0]; Tout[1] = -T0[1]; Tout[2] = -T0[2];
        build_dome(op, cd->node[0].pos,   Tout, R[0],   U[0],   r0, sides, &ring[0]);
        build_dome(op, cd->node[n-1].pos, Tn,   R[n-1], U[n-1], rn, sides, &ring[(n-1)*sides]);
    }

    /* Optional centreline spine on the NEXT (core) layer, emitted as part of the
       SAME build as the tube (this is the mechanism that actually worked in the
       early versions). Curve and/or 2pt-poly; both commit when the tube does. */
    if ((cd->mkCurve || cd->mkPoly) && op->setLayer) {
        LWPntID *sp = (LWPntID *)malloc(sizeof(LWPntID) * (size_t)n);
        if (sp) {
            int base = op->layerNum, ok = 1;
            op->setLayer(op->state, base + SPINE_LAYER_OFFSET);
            for (i = 0; i < n; i++) { sp[i] = addPt(op, cd->node[i].pos); if (!sp[i]) ok = 0; }
            if (ok) {
                if (cd->mkCurve) op->addCurve(op->state, SURFACE_NAME, n, sp, 0);
                if (cd->mkPoly)
                    for (i = 0; i < n-1; i++) { LWPntID seg[2]; seg[0]=sp[i]; seg[1]=sp[i+1]; addFc(op, SURFACE_NAME, 2, seg); }
            }
            op->setLayer(op->state, base);
            free(sp);
        }
    }

    free(ring); ring = NULL;
    }                                            /* end symmetry pass loop */
    g_mir = 0;

    /* On a bake (cd->baking, set on tool-drop) ACCEPT keeps the tube + any spine.
       Otherwise this was a live preview rebuild -> NOTHING. */
    cd->update = cd->baking ? LWT_TEST_ACCEPT : LWT_TEST_NOTHING;
    return NULL;
}

/* ================================================================= */
/* Tool callbacks                                                    */
/* ================================================================= */

/* Framework handles. Reporting them via count()/handle() lets MODELER pick &
   drag them NATIVELY in every viewport (this is how the SDK capsule/seashell
   tools work in all views). Block layout is defined at fw_count()/fw_pos():
       node positions | scale | twist | branch tips.                         */
static int Count(void *inst, LWToolEvent *e)
{
    CurveData *cd = (CurveData *)inst; (void)e;
    return fw_count(cd);
}

/* First-ever click (count==0): drop the initial 2-node segment and drag B. */
static int Start(void *inst, LWToolEvent *e)
{
    CurveData *cd = (CurveData *)inst;
    if (cd->nnodes != 0) return -1;
    v_copy(e->posSnap, cd->node[0].pos); cd->node[0].radius = cd->thickness; cd->node[0].roll = 0.0;
    v_copy(e->posSnap, cd->node[1].pos); cd->node[1].radius = cd->thickness; cd->node[1].roll = 0.0;
    cd->nnodes = 2;
    cd->dirty = 1; cd->update = LWT_TEST_UPDATE;
    return 1;                                    /* framework drags node[1] */
}

static int Handle(void *inst, LWToolEvent *e, int i, LWDVector pos)
{
    CurveData *cd = (CurveData *)inst; (void)e;
    return fw_pos(cd, i, pos);
}

static int Adjust(void *inst, LWToolEvent *e, int i)
{
    CurveData *cd = (CurveData *)inst; int n = cd->nnodes;
    double vax[3];
    if (i < 0) return i;
    safe_axis(e->axis, vax);
    if (i < n) {                                 /* move node (keep depth) */
        node_to_cursor(cd->node[i].pos, e->posSnap, vax);
    } else if (i < 2*n) {                         /* scale node */
        int k = i - n;
        double r = pick_dist(e->posSnap, cd->node[k].pos, vax);
        cd->node[k].radius = (r < MINR) ? MINR : r;
    } else if (i < 3*n) {                          /* twist node about its axis */
        int k = i - 2*n;
        double Rb[MAX_NODES][3], Ub[MAX_NODES][3], T[3], perp[3];
        compute_frames(cd, Rb, Ub);              /* BASE (un-rolled) frame */
        node_tangent(cd, k, T);
        v_sub(e->posSnap, cd->node[k].pos, perp);
        v_projperp(perp, T, perp);               /* into the ring plane */
        if (v_len(perp) > 1e-9)                   /* handle lies along rolled U */
            cd->node[k].roll = atan2(-v_dot(perp, Rb[k]), v_dot(perp, Ub[k]));
    } else return i;
    cd->dirty = 1; cd->update = LWT_TEST_UPDATE;
    return i;
}

/* Is the click on/near ANY framework handle (node/scale/twist)?
   Measured perpendicular to the view axis so it's the same in every view. */
static int near_handle(CurveData *cd, const LWToolEvent *e,
                       const double *vax, double tol)
{
    int i, c = fw_count(cd);
    double hp[3];
    for (i = 0; i < c; i++)
        if (fw_pos(cd, i, hp) && pick_dist(e->posRaw, hp, vax) <= tol) return 1;
    return 0;
}

/* Raw mouse-down. Returns 0 to hand the click to MODELER's native handle
   picker (moves an existing handle in ANY viewport); returns 1 to do our own
   create / extend / split and drive it via move()/up(). */
static int Down(void *inst, LWToolEvent *e)
{
    CurveData *cd = (CurveData *)inst;
    double px = v_len(e->ax), tol, proj[3], vax[3];
    int best;

    if (px < 1e-9) px = 1e-4;
    tol = px * 10.0;                             /* ~10 px pick radius */
    safe_axis(e->axis, vax);                     /* view depth axis (unit) */

    /* No curve yet -> let the framework's Start() create the first segment. */
    if (cd->nnodes == 0) return 0;

    /* Click on an existing handle -> hand it to Modeler's native picker, which
       grabs & drags it correctly in EVERY viewport (this is the multi-view
       fix). We return 0 so adjust() — not our move() — runs. */
    if (near_handle(cd, e, vax, tol)) { cd->drag = -1; return 0; }

    /* On a segment -> split there (raw). drag stores the NODE index. */
    best = find_split(cd, e->posSnap, vax, proj);
    if (best >= 0 && cd->nnodes < MAX_NODES) {
        int j, ins = best + 1;
        double r0 = cd->node[best].radius, r1 = cd->node[best+1].radius;
        for (j = cd->nnodes; j > ins; j--) cd->node[j] = cd->node[j-1];
        v_copy(proj, cd->node[ins].pos);
        cd->node[ins].radius = (r0 + r1) * 0.5;
        cd->node[ins].roll   = (cd->node[best].roll + cd->node[best+1].roll) * 0.5;
        cd->nnodes++;
        cd->drag = ins;
        cd->dirty = 1; cd->update = LWT_TEST_UPDATE;
        return 1;
    }

    /* Otherwise extend: append a node and drag it (raw). The new node keeps
       the LAST node's thickness so a scaled chain stays scaled. */
    if (cd->nnodes < MAX_NODES) {
        v_copy(e->posSnap, cd->node[cd->nnodes].pos);
        cd->node[cd->nnodes].radius = cd->node[cd->nnodes - 1].radius;
        cd->node[cd->nnodes].roll   = cd->node[cd->nnodes - 1].roll;
        cd->nnodes++;
        cd->drag = cd->nnodes - 1;
        cd->dirty = 1; cd->update = LWT_TEST_UPDATE;
        return 1;
    }

    cd->drag = -1;
    return 1;
}

/* Raw drag — only for create/extend/split (cd->drag = node index). Moving
   EXISTING handles goes through adjust() instead. Absolute in-plane positioning:
   follows the cursor in this view, keeps depth from other views, resize-safe. */
static void Move(void *inst, LWToolEvent *e)
{
    CurveData *cd = (CurveData *)inst;
    double vax[3];
    safe_axis(e->axis, vax);
    if (cd->drag >= 0 && cd->drag < cd->nnodes)
        node_to_cursor(cd->node[cd->drag].pos, e->posSnap, vax);
    else return;
    cd->dirty = 1; cd->update = LWT_TEST_UPDATE;
}

static void Up(void *inst, LWToolEvent *e)
{
    CurveData *cd = (CurveData *)inst; (void)e;
    cd->drag = -1;
}

/* Keyboard. TAB subdivides the chain by inserting a midpoint node between every
   pair (keeps every handle live) and BLOCKS Modeler's default Tab. */
static int Key(void *inst, LWDualKey key)
{
    CurveData *cd = (CurveData *)inst;
    if (key == KEY_TAB) {
        int oldn = cd->nnodes, i;
        if (oldn >= 2 && (oldn*2 - 1) <= MAX_NODES) {
            for (i = oldn - 1; i > 0; i--) {
                cd->node[2*i] = cd->node[i];             /* original -> even slot */
                cd->node[2*i-1].pos[0] = 0.5*(cd->node[i].pos[0] + cd->node[i-1].pos[0]);
                cd->node[2*i-1].pos[1] = 0.5*(cd->node[i].pos[1] + cd->node[i-1].pos[1]);
                cd->node[2*i-1].pos[2] = 0.5*(cd->node[i].pos[2] + cd->node[i-1].pos[2]);
                cd->node[2*i-1].radius = 0.5*(cd->node[i].radius + cd->node[i-1].radius);
                cd->node[2*i-1].roll   = 0.5*(cd->node[i].roll   + cd->node[i-1].roll);
            }
            cd->nnodes = oldn*2 - 1;
            cd->dirty = 1; cd->update = LWT_TEST_UPDATE;
        }
        return 1;                                        /* handled: no copy */
    }
    return 0;                                            /* let Modeler handle it */
}

static int Dirty(void *inst)
{
    CurveData *cd = (CurveData *)inst;
    return cd->dirty ? (LWT_DIRTY_WIREFRAME | LWT_DIRTY_HELPTEXT) : 0;
}

static int Test(void *inst)
{
    CurveData *cd = (CurveData *)inst;
    return cd->update;
}

static void Draw(void *inst, LWWireDrawAccess *d)
{
    CurveData *cd = (CurveData *)inst;
    double R[MAX_NODES][3], U[MAX_NODES][3];
    double rad = (d->pxScale > 0.0) ? d->pxScale * 4.0 : 0.0;
    LWFVector f;
    int i;

    if (cd->nnodes < 1) return;

    /* chain spine */
    f[0]=(float)cd->node[0].pos[0]; f[1]=(float)cd->node[0].pos[1]; f[2]=(float)cd->node[0].pos[2];
    d->moveTo(d->data, f, LWWIRE_SOLID);
    for (i = 1; i < cd->nnodes; i++) {
        f[0]=(float)cd->node[i].pos[0]; f[1]=(float)cd->node[i].pos[1]; f[2]=(float)cd->node[i].pos[2];
        d->lineTo(d->data, f, LWWIRE_ABSOLUTE);
    }

    /* position-handle markers */
    for (i = 0; i < cd->nnodes; i++) {
        f[0]=(float)cd->node[i].pos[0]; f[1]=(float)cd->node[i].pos[1]; f[2]=(float)cd->node[i].pos[2];
        d->moveTo(d->data, f, LWWIRE_SOLID);
        if (rad > 0.0) d->circle(d->data, rad, 0);
    }

    /* scale (along rolled R) and twist (along rolled U) handles */
    if (cd->showHandles || cd->showTwist) {
        compute_frames(cd, R, U);
        for (i = 0; i < cd->nnodes; i++) {
            double Rr[3], Ur[3], r = cd->node[i].radius, h[3];
            if (r < MINR) r = MINR;
            v_copy(R[i], Rr); v_copy(U[i], Ur);
            roll_frame(Rr, Ur, cd->node[i].roll);
            f[0]=(float)cd->node[i].pos[0]; f[1]=(float)cd->node[i].pos[1]; f[2]=(float)cd->node[i].pos[2];
            if (cd->showHandles) {               /* scale handle */
                v_madd(cd->node[i].pos, Rr, r, h);
                d->moveTo(d->data, f, LWWIRE_DASH);
                { LWFVector g={(float)h[0],(float)h[1],(float)h[2]};
                  d->lineTo(d->data, g, LWWIRE_ABSOLUTE);
                  d->moveTo(d->data, g, LWWIRE_SOLID);
                  if (rad > 0.0) d->circle(d->data, rad*0.75, 0); }
            }
            if (cd->showTwist) {                 /* twist handle */
                v_madd(cd->node[i].pos, Ur, r*1.5, h);
                d->moveTo(d->data, f, LWWIRE_DASH);
                { LWFVector g={(float)h[0],(float)h[1],(float)h[2]};
                  d->lineTo(d->data, g, LWWIRE_ABSOLUTE);
                  d->moveTo(d->data, g, LWWIRE_SOLID);
                  if (rad > 0.0) d->circle(d->data, rad*0.6, 0); }
            }
        }
    }

    cd->dirty = 0;
}

static const char *Help(void *inst, LWToolEvent *e)
{
    (void)inst; (void)e;
    return "TubularBrains: drag handles (move/scale/twist), LEFT-click empty to "
           "add/extend, click a segment to split, TAB subdivides; drop the tool to "
           "bake. Enable Make Curve / 2pt Poly to also emit a spine on the next layer.";
}

static void Event(void *inst, int code)
{
    CurveData *cd = (CurveData *)inst;
    switch (code) {
        case LWT_EVENT_DROP:
            /* Dropping the tool BAKES the tube (the reliable, native commit
               path). Use Clear first if you want to discard instead. */
            if (cd->nnodes >= 2) { cd->baking = 1; cd->update = LWT_TEST_ACCEPT; }
            break;
        case LWT_EVENT_RESET:
            cd->thickness   = 0.05;
            cd->sides       = 12;
            cd->caps        = CAP_FLAT;
            cd->showHandles = 1;
            cd->showTwist   = 0;
            cd->nnodes      = 0;
            cd->drag        = -1;
            cd->wmap        = 0;
            cd->symmetry    = 0;
            cd->mkCurve     = 0;
            cd->mkPoly      = 0;
            cd->baking      = 0;
            cd->dirty       = 1;
            cd->update      = LWT_TEST_REJECT;
            break;
        case LWT_EVENT_ACTIVATE:
            cd->dirty = 1;
            if (cd->nnodes > 0) cd->update = LWT_TEST_UPDATE;
            break;
    }
}

/* Operation finished. A spine bake just made the curve/poly -> mark it baked
   and bring the tube preview back (don't wipe). A tube bake (drop) -> reset for
   a fresh tube. Otherwise leave the chain so editing/handles persist. */
static void End(void *inst, int keep)
{
    CurveData *cd = (CurveData *)inst;
    (void)keep;
    cd->drag = -1;
    cd->update = LWT_TEST_NOTHING;
    if (cd->baking) {                            /* tube (+ spine) baked on drop */
        cd->nnodes = 0;
        cd->baking = 0;
    }
}

static void Done(void *inst) { free(inst); }

/* ================================================================= */
/* Numeric panel                                                     */
/* ================================================================= */
static void *PanelGet(void *inst, unsigned int vid)
{
    CurveData *cd = (CurveData *)inst;
    switch (vid) {
        case ID_THICK: return &cd->thickness;
        case ID_SIDES: return &cd->sides;
        case ID_CAPS:  return &cd->caps;
        case ID_SHOWH: return &cd->showHandles;
        case ID_TWIST: return &cd->showTwist;
        case ID_WMAP:  return &cd->wmap;
        case ID_SYM:   return &cd->symmetry;
        case ID_BAKEC: return &cd->mkCurve;
        case ID_BAKEP: return &cd->mkPoly;
        default:       return NULL;
    }
}

static LWXPRefreshCode PanelSet(void *inst, unsigned int vid, void *value)
{
    CurveData *cd = (CurveData *)inst;
    switch (vid) {
        case ID_THICK: cd->thickness   = *(double *)value; break;
        case ID_SIDES: cd->sides       = *(int *)value;    break;
        case ID_CAPS:  cd->caps        = *(int *)value;    break;
        case ID_SHOWH: cd->showHandles = *(int *)value;    break;
        case ID_TWIST: cd->showTwist   = *(int *)value;    break;
        case ID_WMAP:  cd->wmap        = *(int *)value;    break;
        case ID_SYM:   cd->symmetry    = *(int *)value;    break;
        case ID_BAKEC: cd->mkCurve     = *(int *)value;    break;
        case ID_BAKEP: cd->mkPoly      = *(int *)value;    break;
        default:       return LWXPRC_NONE;
    }
    if (cd->sides < 3) cd->sides = 3;
    cd->dirty  = 1;
    cd->update = LWT_TEST_UPDATE;
    return LWXPRC_DRAW;
}

static void OnClear(LWXPanelID pan, int cid)
{
    CurveData *cd;
    (void)cid;
    if (!xpanf) return;
    cd = (CurveData *)xpanf->getData(pan, 0);
    if (!cd) return;
    cd->nnodes = 0;
    cd->drag   = -1;
    cd->dirty  = 1;
    cd->update = LWT_TEST_REJECT;
}

static LWXPanelID Panel(void *inst)
{
    CurveData *cd = (CurveData *)inst;
    LWXPanelID panel;

    static const char *caps_list[] = { "None", "Flat", "Rounded", NULL };

    static LWXPanelControl ctl[] = {
        { ID_THICK, "Thickness",       "distance" },
        { ID_SIDES, "Sides",           "integer"  },
        { ID_CAPS,  "Caps",            "iChoice"  },
        { ID_WMAP,  "Tube Weight Map", "iBoolean" },
        { ID_SYM,   "Symmetry (X)",    "iBoolean" },
        { ID_SHOWH, "Scale Handles",   "iBoolean" },
        { ID_TWIST, "Twist Handles",   "iBoolean" },
        { ID_BAKEC, "Make Curve",      "iBoolean" },
        { ID_BAKEP, "Make 2pt Poly",   "iBoolean" },
        { ID_CLEAR, "Clear",           "vButton"  },
        { 0 }
    };
    static LWXPanelDataDesc cdata[] = {
        { ID_THICK, "Thickness",       "float"   },
        { ID_SIDES, "Sides",           "integer" },
        { ID_CAPS,  "Caps",            "integer" },
        { ID_WMAP,  "Tube Weight Map", "integer" },
        { ID_SYM,   "Symmetry (X)",    "integer" },
        { ID_SHOWH, "Scale Handles",   "integer" },
        { ID_TWIST, "Twist Handles",   "integer" },
        { ID_BAKEC, "Make Curve",      "integer" },
        { ID_BAKEP, "Make 2pt Poly",   "integer" },
        { 0 }
    };
    LWXPanelHint hint[] = {
        XpLABEL(0, "TubularBrains"),
        XpMIN(ID_SIDES, 3),
        XpSTRLIST(ID_CAPS, caps_list),
        XpBUTNOTIFY(ID_CLEAR, OnClear),
        XpEND
    };

    if (!xpanf) return NULL;

    panel = xpanf->create(LWXP_VIEW, ctl);
    if (!panel) return NULL;

    xpanf->describe(panel, cdata, PanelGet, PanelSet);
    xpanf->hint(panel, 0, hint);
    xpanf->viewInst(panel, cd);
    xpanf->setData(panel, 0, cd);

    return panel;
}

/* ================================================================= */
/* Activation                                                        */
/* ================================================================= */
XCALL_(int)
Activate(int version, GlobalFunc *global, void *inst, void *serverData)
{
    LWMeshEditTool *local = (LWMeshEditTool *)inst;
    CurveData *cd;
    (void)serverData;

    if (version != LWMESHEDITTOOL_VERSION)
        return AFUNC_BADVERSION;

    xpanf = (LWXPanelFuncs *)global(LWXPANELFUNCS_GLOBAL, GFUSE_TRANSIENT);

    cd = (CurveData *)calloc(1, sizeof(CurveData));
    if (!cd) return AFUNC_OK;
    cd->thickness   = 0.05;       /* default 50 mm radius */
    cd->sides       = 12;
    cd->caps        = CAP_FLAT;
    cd->showHandles = 1;
    cd->showTwist   = 0;
    cd->nnodes      = 0;
    cd->drag        = -1;
    cd->wmap        = 0;
    cd->symmetry    = 0;
    cd->update      = LWT_TEST_NOTHING;

    local->instance = cd;

    local->tool->done   = Done;
    local->tool->help   = Help;
    local->tool->count  = Count;
    local->tool->handle = Handle;
    local->tool->start  = Start;
    local->tool->adjust = Adjust;
    local->tool->down   = Down;
    local->tool->move   = Move;
    local->tool->up     = Up;
    local->tool->key    = Key;
    local->tool->draw   = Draw;
    local->tool->dirty  = Dirty;
    local->tool->event  = Event;
    local->tool->panel  = Panel;

    local->test  = Test;
    local->build = Build;
    local->end   = End;

    return AFUNC_OK;
}

/* ---- server record --------------------------------------------- */
ServerRecord ServerDesc[] = {
    { LWMESHEDITTOOL_CLASS, "TubularBrains", Activate },
    { NULL }
};
