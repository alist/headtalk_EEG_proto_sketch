enum SystemStatus {
  unknownStatus = 0,
  faultStatus,
  goodStatus
};

enum SessionState{
  pendingState = 0,
  estopState,
  idleState,
  beginState,
  activeState,
  endState
};

const int D10 = 30;
const int D11 = 12;
