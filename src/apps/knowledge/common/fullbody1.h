#ifndef COMMON_FULLBODY1_H
#define COMMON_FULLBODY1_H

namespace fullbody1 {
    enum {
        root = 0,
        l_thigh = 1,
        l_shin = 2,
        l_heel = 3,
        l_toe = 4,
        r_thigh = 5,
        r_shin = 6,
        r_heel = 7,
        r_toe = 8,
        abdomen = 9,
        spine = 10,
        head = 11,
        l_scapula = 12,
        l_bicep = 13,
        l_forearm = 14,
        l_hand = 15,
        r_scapula = 16,
        r_bicep = 17,
        r_forearm = 18,
        r_hand = 19,
        NUM_NODES = 20,
    };
        
    enum {
        pelvis_tx = 0,
        pelvis_ty = 1,
        pelvis_tz = 2,
        pelvis_ez = 3,
        pelvis_ey = 4,

        pelvis_ex = 5,
        l_thigh_ez,
        l_thigh_ex,
        l_knee_ez,
        l_ankle_ez,

        l_ankle_ey,
        l_toe_ez,
        r_thigh_ez,
        r_thigh_ex,
        r_knee_ez,

        r_ankle_ez,
        r_ankle_ey,
        r_toe_ez,
        abdomen_ex,
        abdomen_ez,

        spine_ey,
        neck_ez,
        neck_ex,
        l_scapula_ex,
        // l_bicep_ez,
        l_bicep_ey,

        l_bicep_ex,
        l_elbow_ez,
        l_wrist_ex,
        r_scapula_ex,
        // r_bicep_ez,
        r_bicep_ey,

        r_bicep_ex,
        r_elbow_ez,
        r_wrist_ex,
        NUM_DOFS
    };
} // namespace fullbody1

#endif // #ifndef COMMON_FULLBODY1_H
