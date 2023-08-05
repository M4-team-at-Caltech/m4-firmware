int32 CMD_STAND=0
int32 CMD_CROUCH=1
int32 CMD_SIT=2
int32 CMD_UAV=3
int32 CMD_MIP=4
int32 CMD_CUSTOM=5

int32 commandID

# Used only by CMD_CUSTOM. Otherwise ignored
int32[] ids # Which joint IDs to move				
float64[] joint_positions # Joint Positions in degrees to move to