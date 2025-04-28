#ifndef MODE_H
#define MODE_H

enum Mode {
        MANUAL,
        ACRO,
        STAB,
        //PHOTO_MAX_HOLD,
        //USER
} current_mode = STAB;


inline const char* getModeName(Mode m) {
    static const char* names[] = {"MANUAL", "ACRO", "STAB"};
    return names[m];
}

extern Mode current_mode;
#endif
