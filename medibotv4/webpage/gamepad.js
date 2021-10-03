const refreshRate = 10;
var pressed = 0;

window.addEventListener("gamepadconnected", (event) => {
  console.log("A gamepad connected");
  repGP = window.setInterval(getGamepadState, refreshRate);
});

window.addEventListener("gamepaddisconnected", (event) => {
  console.log("A gamepad disconnected");
  window.clearInterval(repGP);
});


function getGamepadState(){
   // Returns up to 4 gamepads.
  const gamepads = navigator.getGamepads();

  // We take the first one, for simplicity
  const gamepad = gamepads[0];

  // Escape if no gamepad was found
  if (!gamepad) {
    console.log('No gamepad found.');
    return;
  }

  if(gamepad.id=='046d-c21d-Logitech Gamepad F310'){
    //LOGITECH F310 CONTROLLER - START
    //AXES AND BUTTONS ASSIGNMENT
    //AXIS 6: DPAD LEFT/RIGHT, AXIS 7: DPAD UP/DOWN
    //B0: A, B1: B, B2: X, B3:Y
    var xAxis = gamepad.axes[6];
    var yAxis = gamepad.axes[7];
    var aButton = gamepad.buttons[0];
    var bButton = gamepad.buttons[1];
    var xButton = gamepad.buttons[2];
    var yButton = gamepad.buttons[3];

    if(yAxis<0.0){
      console.log('forward');
      moveAction(0.5, 0.0);
    }
    else if(yAxis>0.0){
      console.log('reverse');
      moveAction(-0.5, 0.0);
    }
    else if(xAxis<0.0){
      console.log('left');
      moveAction(0.0, 1.0);
    }
    else if(xAxis>0.0){
      console.log('right');
      moveAction(0.0, -1.0);
    }
    else if(buttonPressed(yButton)){
      yButtonChange();
    }
    else if(buttonPressed(xButton)){
      xButtonChange();
    }
    else if(buttonPressed(aButton)){
      aButtonChange();
    }
    else if(buttonPressed(bButton)){
      bButtonChange();
    }
    else{
      console.log('stop');
      moveAction(0.0, 0.0);
    }
    //LOGITECH F310 CONTROLLER - END
  }
  else if(gamepad.id=='046d-c215-Logitech Logitech Extreme 3D'){
    //LOGITECH X3D JOYSTICK - START
    var xAxis = gamepad.axes[0];
    var yAxis = gamepad.axes[1];
    var pointTurnAxis = gamepad.axes[2];

    if (buttonPressed(gamepad.buttons[0])) {
      if (yAxis < -0.1 && pointTurnAxis < 0.5 && pointTurnAxis > -0.5){
        console.log('forward');
        moveAction(0.5, 0.0);
      }
      else if (yAxis > 0.1 && pointTurnAxis < 0.8 && pointTurnAxis > -0.8){
        console.log('reverse');
        moveAction(-0.5, 0.0);
      }
      else if (pointTurnAxis > 0.8){
        console.log('right');
        moveAction(0.0, -1.0);
      }
      else if (pointTurnAxis < -0.8){
        console.log('left');
        moveAction(0.0, 1.0);
      }
      else{
        console.log('stop pressed');
        moveAction(0.0, 0.0);
      }
    }
    else if(buttonPressed(gamepad.buttons[4])){
      yButtonChange();
    }
    else if(buttonPressed(gamepad.buttons[2])){
      xButtonChange();
    }
    else if(buttonPressed(gamepad.buttons[3])){
      aButtonChange();
    }
    else if(buttonPressed(gamepad.buttons[5])){
      bButtonChange();
    }
    else {
      console.log('stop');
      moveAction(0.0, 0.0);
    }
    //LOGITECH X3D JOYSTICK - END
  }
}


function buttonPressed(b) {

  if (typeof (b) == "object") {
    return b.pressed;
  }
  return b == 1.0;
}

function debounce(func, timeout = 300){
  let timer;
  return (...args) => {
    clearTimeout(timer);
    timer = setTimeout(() => { func.apply(this, args); }, timeout);
  };
}

const yButtonChange = debounce(() => changePwm(10,0));
const xButtonChange = debounce(() => changePwm(-10,0));
const bButtonChange = debounce(() => changePwm(0,10));
const aButtonChange = debounce(() => changePwm(0,-10));



