# This is the error code every skill action returns, so a caller can tell one kind of failure
# from another and decide what to do with it, and so a log line says more than "failed".
uint8 SUCCESS=0
uint8 PLAN_FAILED=1        # MoveIt found no plan, or the Cartesian path was incomplete
uint8 EXECUTION_FAILED=2   # trajectory or gripper execution failed
uint8 GRASP_FAILED=3       # gripper closed on nothing, or the object slipped out
uint8 RELEASE_FAILED=4     # gripper did not open
uint8 INSERT_JAMMED=5      # reserved; no skill returns it yet
uint8 TIMEOUT=6
uint8 CANCELED=7
uint8 INVALID_GOAL=8       # malformed goal (unknown frame, bad size, ...)
uint8 UNKNOWN_TARGET=9     # target name not in the workspace
uint8 NOT_HOLDING=10       # Place without a successful Pick
uint8 NOT_READY=11         # skill server still initialising

uint8 code
