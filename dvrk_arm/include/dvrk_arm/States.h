#ifndef CDVRK_StatesH
#define CDVRK_StatesH
struct States{
public:
    States(){
        mStates[DVRK_UNINITIALIZED] = "DVRK_UNINITIALIZED";
        mStates[DVRK_POSITION_JOINT]= "DVRK_POSITION_JOINT";
        mStates[DVRK_POSITION_CARTESIAN] = "DVRK_POSITION_CARTESIAN";
        mStates[DVRK_EFFORT_CARTESIAN] = "DVRK_EFFORT_CARTESIAN";


        _m_effort_mode = mStates[DVRK_EFFORT_CARTESIAN];
        _m_jnt_pos_mode = mStates[DVRK_POSITION_JOINT];
        _m_cart_pos_mode = mStates[DVRK_POSITION_CARTESIAN];

        activeState = DVRK_UNINITIALIZED;
    }

    enum eSTATES{DVRK_UNINITIALIZED,
                DVRK_POSITION_CARTESIAN,
                DVRK_POSITION_JOINT,
                DVRK_EFFORT_CARTESIAN};
    eSTATES activeState;

    std::map<eSTATES, std::string> mStates;

    std::string _m_effort_mode;
    std::string _m_jnt_pos_mode;
    std::string _m_cart_pos_mode;
};

#endif
