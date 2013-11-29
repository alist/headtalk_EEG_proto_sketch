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


const int D2 = 19;
const int D3 = 18;

const int D9 = 29;
const int D10 = 30;
const int D11 = 12;
