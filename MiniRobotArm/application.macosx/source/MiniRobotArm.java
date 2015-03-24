import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import de.voidplus.leapmotion.*; 
import g4p_controls.*; 
import processing.serial.*; 

import java.util.HashMap; 
import java.util.ArrayList; 
import java.io.File; 
import java.io.BufferedReader; 
import java.io.PrintWriter; 
import java.io.InputStream; 
import java.io.OutputStream; 
import java.io.IOException; 

public class MiniRobotArm extends PApplet {


// Need G4P library



Serial myPort;
String s="";
PImage crciberneticalogo;
LeapMotion leap;

float A = 50.0f; //millimeters
float B = 50.0f;
float rtod = 57.295779f;

boolean sval=false;

GImageToggleButton btnToggle0;
GImageToggleButton btnToggle1;

PrintWriter output;

String fileName;
Table table;
String selected_file;
int selectedfilecount=0;

boolean LEAPMOTION = false;
boolean PLAYBACK = false;
boolean RECORD = false;
int playback_count = 0;
long previousMillis = 0;
long previousBlinkMillis = 0;
long interval = 100;
boolean LAMP = false;

public void setup() {
  size(750, 700, JAVA2D);
  createGUI();
  btnToggle0 = new GImageToggleButton(this, 133, 469);
  btnToggle0.tag = "0";
  btnToggle1 = new GImageToggleButton(this, 183, 469);
  btnToggle1.tag = "1";
  leap = new LeapMotion(this);
  crciberneticalogo = loadImage("CRCibernetica509x81.png");
  String portName = "";
  myPort = new Serial(this, portName, 38400);
  myPort.bufferUntil('\n');

  dropList1.setItems(Serial.list(), 0);
  fileName = getDateTime();
  output = createWriter("data/" + "positions" + fileName + ".csv");
  output.println("x,y,z,g,wa,wr");
}

public void draw() {
  background(255);
  image(crciberneticalogo, 414, 636, 305, 48.5f);

  if (LEAPMOTION) {
    updateLeapMotion();
  }
  updatePlayBack();
  updateAnimation();
  
 // updateBlink();
}

/* arm positioning routine utilizing inverse kinematics */
/* z is base angle, y vertical distance from base, x is horizontal distance.*/
public int Arm(float x, float y, float z, int g, float wa, int wr)
{
  float M = sqrt((y*y)+(x*x));
  if (M <= 0)
    return 1;
  float A1 = atan(y/x);
  float A2 = acos((A*A-B*B+M*M)/((A*2)*M));
  float Elbow = acos((A*A+B*B-M*M)/((A*2)*B));
  float Shoulder = A1 + A2;
  Elbow = Elbow * rtod;
  Shoulder = Shoulder * rtod;
  while ( (int)Elbow <= 0 || (int)Shoulder <= 0)
    return 1;
  float Wris = abs(wa - Elbow - Shoulder) - 90;
  //slider1.setValue(z);
  slider2.setValue(180-Shoulder);
  slider3.setValue(180-Elbow);
  slider4.setValue(Wris);
  //slider5.setValue(g);
  return 0;
}

// Event handler for image toggle buttons
public void handleToggleButtonEvents(GImageToggleButton button, GEvent event) { 
  println(button + "   State: " + button.stateValue());
  if (button.tag=="1") {
    LEAPMOTION=PApplet.parseBoolean(button.stateValue());
  }
  if (button.tag=="0") {
    //toggle main light
    //println("Light: " + button.stateValue());
    //controlLamp(button.stateValue());
  }
}
public void keyPressed() {
  if (keyCode==16){//right shift
    PLAYBACK = false;
    playback_count = 0;
    selected_file="normalstance.csv";
    PLAYBACK=true;
  }
   if (keyCode==47){// forward slash
    PLAYBACK = false;
    playback_count = 0;
    selectedfilecount=selectedfilecount+1;
    if(selectedfilecount>1){
      selectedfilecount=1;
    }
    selected_file=selectedfilecount+".csv";
    PLAYBACK=true;
    //controlEyes(1);
    //controlLamp(0);
  }
  
  if (keyCode==32) {
    sval=!sval;
    int stateVal=PApplet.parseInt(sval);
    btnToggle0.stateValue(stateVal);
    slider5.setValue((stateVal*130));
  }
  if (keyCode==83) { // s to save coordinates to file
    println("Coordinates saved to file");
    float x = slider2d1.getValueXI();
    float y = slider2d1.getValueYI();
    float z = slider1.getValueI();
    int g = slider5.getValueI();
    float w = slider6.getValueI();
    output.println(x + "," + y + "," + z + "," + g + "," + w + "," + 90);
  }
  if (keyCode==88) {
    println("Close file"); //x to save and close file
    RECORD=false;
    output.flush(); // Writes the remaining data to the file
    output.close(); // Finishes the file
    //exit(); // Stops the program
  }
  if (keyCode==80) {
    println("Playback"); //p for playback
    PLAYBACK = true;
  }
  if (keyCode==78) { 
    fileName = getDateTime();
    output = createWriter("data/" + "positions" + fileName + ".csv");
    output.println("x,y,z,g,wa,wr");
    println("New position file"); //n for new file
  }
  if (keyCode==79) {
    println("Open File for Playback"); //o for open file for playback
    selectInput("Select a file to playback:", "fileSelected");
  }
  if (keyCode==82) {
    println("Record"); //r for record
    RECORD = true;
  }
}

public void updateLeapMotion() {
  int fps = leap.getFrameRate();
  // HANDS
  // for(Hand hand : leap.getHands()){
  ArrayList hands = leap.getHands();
  if (!hands.isEmpty()) {
    Hand hand1 = (Hand) hands.toArray()[0];
    //Hand hand2 = (Hand) hands.toArray()[1];

    //hand.draw();
    //int     hand_id          = hand.getId();
    PVector hand1_position    = hand1.getPosition();
    PVector hand1_stabilized  = hand1.getStabilizedPosition();
    //PVector hand_direction   = hand.getDirection();
    //PVector hand_dynamics    = hand.getDynamics();
    float   hand1_roll        = hand1.getRoll();
    //float   hand1_pitch       = hand1.getPitch();
    float   hand1_yaw         = hand1.getYaw();
    float   hand1_time        = hand1.getTimeVisible();
    //float   hand2_time        = hand2.getTimeVisible();
    //PVector sphere_position  = hand.getSpherePosition();
    //float   sphere_radius    = hand.getSphereRadius();
    if (hand1_time>1.0f) {
      //println("x: " +hand1_stabilized.x+" y: "+hand1_stabilized.y+" z:"+hand1_stabilized.z);
      float transHand1PosZ = map(hand1_stabilized.x, 150, 450, 130, 50);
      slider1.setValue(transHand1PosZ);
      float transHand1PosY = map(hand1_stabilized.y, 550, 300, 20, 110);
      slider2d1.setValueY(transHand1PosY);
      float transHand1PosX = map(hand1_position.z, 30, 55, 20, 110);
      slider2d1.setValueX(transHand1PosX);
      float transHand1Roll = map(hand1_roll, 30, -30, 45, -80);
      //println("HandRoll " +hand_roll+" OUTPUT: "+transHandRoll);
      slider6.setValue(transHand1Roll);
      float transHand1Yaw = map(hand1_yaw, 0, 50, 50, 175);
      println("Yaw " +hand1_yaw+" OUTPUT: "+transHand1Yaw);
     //  slider5.setValue(transHand1Yaw);
    }
  }
}

public void fileSelected(File selection) {
  if (selection == null) {
    println("Window was closed or the user hit cancel.");
  } 
  else {
    println("User selected " + selection.getAbsolutePath());
    selected_file = selection.getAbsolutePath();
  }
}

public String getDateTime() {
  int d = day();  
  int m = month(); 
  int y = year();  
  int h = hour();
  int min = minute();
  String s = String.valueOf(y);
  s = s + String.valueOf(m);
  s = s + String.valueOf(d);
  s = s + String.valueOf(h);
  s = s + String.valueOf(min);
  return s;
}
public int ArmPlayBack(float x, float y, float z, int g, float wa, int wr)
{
  float M = sqrt((y*y)+(x*x));
  if (M <= 0)
    return 1;
  float A1 = atan(y/x);
  float A2 = acos((A*A-B*B+M*M)/((A*2)*M));
  float Elbow = acos((A*A+B*B-M*M)/((A*2)*B));
  float Shoulder = A1 + A2;
  Elbow = Elbow * rtod;
  Shoulder = Shoulder * rtod;
  while ( (int)Elbow <= 0 || (int)Shoulder <= 0)
    return 1;
  float Wris = abs(wa - Elbow - Shoulder) - 90;
  slider1.setValue(z);
  slider2.setValue(180-Shoulder);
  slider3.setValue(180-Elbow);
  slider4.setValue(Wris);
  slider5.setValue(g);
  return 0;
}

public void saveCoordinates() {
  float x = slider2d1.getValueXI();
  float y = slider2d1.getValueYI();
  float z = slider1.getValueI();
  int g = slider5.getValueI();
  float w = slider6.getValueI();
  output.println(x + "," + y + "," + z + "," + g + "," + w + "," + 90);
}

public void controlEyes(int eyes){
  int x = 255*eyes;
  myPort.clear();
  myPort.write("8,"+0+","+x+","+0+"\n");
  myPort.write("9,"+0+","+x+","+0+"\n");
}
public void controlLamp(int lamp){
  int x = 255*lamp;
  myPort.clear();
  myPort.write("6,"+x+","+x+","+x+"\n");
  myPort.write("7,"+x+","+x+","+x+"\n");
  myPort.write("10,"+x+","+x+","+x+"\n");
  myPort.write("11,"+x+","+x+","+x+"\n");
}
/* =========================================================
 * ====                   WARNING                        ===
 * =========================================================
 * The code in this tab has been generated from the GUI form
 * designer and care should be taken when editing this file.
 * Only add/edit code inside the event handlers i.e. only
 * use lines between the matching comment tags. e.g.

 void myBtnEvents(GButton button) { //_CODE_:button1:12356:
     // It is safe to enter your event code here  
 } //_CODE_:button1:12356:
 
 * Do not rename this tab!
 * =========================================================
 */

public void slider1_change1(GSlider source, GEvent event) { //_CODE_:slider1:790724:
  //println("slider1 - GSlider event occured " + System.currentTimeMillis()%10000000 );
  myPort.clear();
  myPort.write("0,"+slider1.getValueI()+"\n");

} //_CODE_:slider1:790724:

public void dropList1_click1(GDropList source, GEvent event) { //_CODE_:dropList1:697391:
  //println("dropList1 - GDropList event occured " + System.currentTimeMillis()%10000000 );
  int i = dropList1.getSelectedIndex();
  myPort.clear();
  myPort.stop();
  myPort = new Serial(this, Serial.list()[i], 38400);
  myPort.bufferUntil('\n');
} //_CODE_:dropList1:697391:

public void slider2_change1(GSlider source, GEvent event) { //_CODE_:slider2:752459:
  //println("slider2 - GSlider event occured " + System.currentTimeMillis()%10000000 );
  myPort.clear();
  myPort.write("1,"+slider2.getValueI()+"\n");
} //_CODE_:slider2:752459:

public void slider3_change1(GSlider source, GEvent event) { //_CODE_:slider3:489757:
  //println("slider3 - GSlider event occured " + System.currentTimeMillis()%10000000 );
  myPort.clear();
  myPort.write("2,"+slider3.getValueI()+"\n");
} //_CODE_:slider3:489757:

public void slider4_change1(GSlider source, GEvent event) { //_CODE_:slider4:801509:
  //println("slider4 - GSlider event occured " + System.currentTimeMillis()%10000000 );
  myPort.clear();
  myPort.write("3,"+slider4.getValueI()+"\n");
} //_CODE_:slider4:801509:

public void slider5_change1(GSlider source, GEvent event) { //_CODE_:slider5:834674:
  //println("slider5 - GSlider event occured " + System.currentTimeMillis()%10000000 );
  myPort.clear();
  myPort.write("4,"+slider5.getValueI()+"\n");
} //_CODE_:slider5:834674:

public void slider2d1_change1(GSlider2D source, GEvent event) { //_CODE_:slider2d1:750043:
  //println("slider2d1 - GSlider2D event occured " + System.currentTimeMillis()%10000000 );
  float x = slider2d1.getValueXI();
  float y = slider2d1.getValueYI();
  label5.setText(""+x);
  label6.setText(""+y);
  float z = slider1.getValueI();
  int g = slider5.getValueI();
  float w = slider6.getValueI();
  Arm(x,y,z,g,w,90);
} //_CODE_:slider2d1:750043:

public void slider6_change1(GSlider source, GEvent event) { //_CODE_:slider6:431756:
  //println("slider6 - GSlider event occured " + System.currentTimeMillis()%10000000 );
} //_CODE_:slider6:431756:

public void button1_click1(GButton source, GEvent event) { //_CODE_:button1:289466:
  selected_file="normalstance.csv";
  PLAYBACK=true;
} //_CODE_:button1:289466:

public void button2_click1(GButton source, GEvent event) { //_CODE_:button2:319793:
  selected_file="sad.csv";
  PLAYBACK=true;
} //_CODE_:button2:319793:

public void button3_click1(GButton source, GEvent event) { //_CODE_:button3:218967:
  selected_file="recording.csv";
  PLAYBACK=true;
} //_CODE_:button3:218967:

public void button4_click1(GButton source, GEvent event) { //_CODE_:button4:923212:
  println("button4 - GButton event occured " + System.currentTimeMillis()%10000000 );
  myPort.clear();
  myPort.write("8,0,100,0\n");
  myPort.write("9,0,100,0\n");
  
} //_CODE_:button4:923212:

public void button5_click1(GButton source, GEvent event) { //_CODE_:button5:467161:
  println("button5 - GButton event occured " + System.currentTimeMillis()%10000000 );
  myPort.clear();
  myPort.write("8,0,0,0\n");
  myPort.write("9,0,0,0\n");
  
} //_CODE_:button5:467161:

public void button6_click1(GButton source, GEvent event) { //_CODE_:button6:927708:
  println("button6 - GButton event occured " + System.currentTimeMillis()%10000000 );
} //_CODE_:button6:927708:

public void button7_click1(GButton source, GEvent event) { //_CODE_:button7:386892:
  println("button7 - GButton event occured " + System.currentTimeMillis()%10000000 );
} //_CODE_:button7:386892:

public void button8_click1(GButton source, GEvent event) { //_CODE_:button8:478806:
  println("button8 - GButton event occured " + System.currentTimeMillis()%10000000 );
} //_CODE_:button8:478806:

public void button9_click1(GButton source, GEvent event) { //_CODE_:button9:853082:
  println("button9 - GButton event occured " + System.currentTimeMillis()%10000000 );
} //_CODE_:button9:853082:



// Create all the GUI controls. 
// autogenerated do not edit
public void createGUI(){
  G4P.messagesEnabled(false);
  G4P.setGlobalColorScheme(GCScheme.BLUE_SCHEME);
  G4P.setCursor(ARROW);
  if(frame != null)
    frame.setTitle("Controlador Mini Brazo");
  slider1 = new GSlider(this, 366, 15, 350, 75, 30.0f);
  slider1.setShowValue(true);
  slider1.setShowLimits(true);
  slider1.setLimits(90.0f, 0.0f, 180.0f);
  slider1.setEasing(10.0f);
  slider1.setNumberFormat(G4P.INTEGER, 0);
  slider1.setOpaque(false);
  slider1.addEventHandler(this, "slider1_change1");
  dropList1 = new GDropList(this, 134, 9, 195, 243, 9);
  dropList1.setItems(loadStrings("list_697391"), 0);
  dropList1.addEventHandler(this, "dropList1_click1");
  slider2 = new GSlider(this, 366, 92, 350, 75, 30.0f);
  slider2.setShowValue(true);
  slider2.setShowLimits(true);
  slider2.setLimits(90.0f, 0.0f, 180.0f);
  slider2.setEasing(10.0f);
  slider2.setNumberFormat(G4P.INTEGER, 0);
  slider2.setOpaque(false);
  slider2.addEventHandler(this, "slider2_change1");
  slider3 = new GSlider(this, 366, 168, 350, 75, 30.0f);
  slider3.setShowValue(true);
  slider3.setShowLimits(true);
  slider3.setLimits(90.0f, 0.0f, 180.0f);
  slider3.setEasing(10.0f);
  slider3.setNumberFormat(G4P.INTEGER, 0);
  slider3.setOpaque(false);
  slider3.addEventHandler(this, "slider3_change1");
  slider4 = new GSlider(this, 365, 244, 350, 75, 30.0f);
  slider4.setShowValue(true);
  slider4.setShowLimits(true);
  slider4.setLimits(90.0f, 0.0f, 180.0f);
  slider4.setEasing(10.0f);
  slider4.setNumberFormat(G4P.INTEGER, 0);
  slider4.setOpaque(false);
  slider4.addEventHandler(this, "slider4_change1");
  slider5 = new GSlider(this, 366, 321, 350, 75, 30.0f);
  slider5.setShowValue(true);
  slider5.setShowLimits(true);
  slider5.setLimits(90.0f, 0.0f, 180.0f);
  slider5.setEasing(10.0f);
  slider5.setNumberFormat(G4P.INTEGER, 0);
  slider5.setOpaque(false);
  slider5.addEventHandler(this, "slider5_change1");
  slider2d1 = new GSlider2D(this, 26, 148, 305, 153);
  slider2d1.setLimitsX(40.0f, 0.0f, 110.0f);
  slider2d1.setLimitsY(40.0f, 0.0f, 110.0f);
  slider2d1.setNumberFormat(G4P.DECIMAL, 0);
  slider2d1.setOpaque(true);
  slider2d1.addEventHandler(this, "slider2d1_change1");
  label1 = new GLabel(this, 22, 9, 103, 27);
  label1.setText("Serial Port");
  label1.setTextBold();
  label1.setOpaque(true);
  label2 = new GLabel(this, 26, 313, 133, 20);
  label2.setText("Distance from origin:");
  label2.setTextAlign(GAlign.LEFT, GAlign.MIDDLE);
  label2.setOpaque(false);
  label3 = new GLabel(this, 96, 340, 56, 17);
  label3.setText("X(mm):");
  label3.setTextAlign(GAlign.RIGHT, GAlign.MIDDLE);
  label3.setOpaque(false);
  label4 = new GLabel(this, 72, 363, 80, 20);
  label4.setText("Y(mm):");
  label4.setTextAlign(GAlign.RIGHT, GAlign.MIDDLE);
  label4.setOpaque(false);
  label5 = new GLabel(this, 159, 340, 80, 20);
  label5.setText("0");
  label5.setOpaque(false);
  label6 = new GLabel(this, 158, 365, 80, 20);
  label6.setText("0");
  label6.setOpaque(false);
  slider6 = new GSlider(this, 366, 441, 350, 75, 30.0f);
  slider6.setShowValue(true);
  slider6.setShowLimits(true);
  slider6.setLimits(0.0f, -90.0f, 90.0f);
  slider6.setEasing(10.0f);
  slider6.setNumberFormat(G4P.INTEGER, 0);
  slider6.setOpaque(false);
  slider6.addEventHandler(this, "slider6_change1");
  button1 = new GButton(this, 32, 536, 80, 30);
  button1.setText("Normal");
  button1.addEventHandler(this, "button1_click1");
  button2 = new GButton(this, 124, 537, 80, 30);
  button2.setText("Sad");
  button2.addEventHandler(this, "button2_click1");
  button3 = new GButton(this, 216, 537, 80, 30);
  button3.setText("Blink");
  button3.addEventHandler(this, "button3_click1");
  button4 = new GButton(this, 33, 577, 80, 30);
  button4.setText("Yes");
  button4.addEventHandler(this, "button4_click1");
  button5 = new GButton(this, 124, 577, 80, 30);
  button5.setText("No");
  button5.addEventHandler(this, "button5_click1");
  button6 = new GButton(this, 215, 578, 80, 30);
  button6.setText("Face text");
  button6.addEventHandler(this, "button6_click1");
  label9 = new GLabel(this, 160, 414, 85, 48);
  label9.setText("Enable LeapMotion");
  label9.setOpaque(false);
  button7 = new GButton(this, 34, 619, 80, 30);
  button7.setText("Face text");
  button7.addEventHandler(this, "button7_click1");
  button8 = new GButton(this, 125, 619, 80, 30);
  button8.setText("Face text");
  button8.addEventHandler(this, "button8_click1");
  button9 = new GButton(this, 216, 619, 80, 30);
  button9.setText("Face text");
  button9.addEventHandler(this, "button9_click1");
  label8 = new GLabel(this, 120, 434, 66, 20);
  label8.setText("Claw");
  label8.setOpaque(false);
}

// Variable declarations 
// autogenerated do not edit
GSlider slider1; 
GDropList dropList1; 
GSlider slider2; 
GSlider slider3; 
GSlider slider4; 
GSlider slider5; 
GSlider2D slider2d1; 
GLabel label1; 
GLabel label2; 
GLabel label3; 
GLabel label4; 
GLabel label5; 
GLabel label6; 
GSlider slider6; 
GButton button1; 
GButton button2; 
GButton button3; 
GButton button4; 
GButton button5; 
GButton button6; 
GLabel label9; 
GButton button7; 
GButton button8; 
GButton button9; 
GLabel label8; 

public void updatePlayBack() {
  long currentMillis = millis();
  if (PLAYBACK&&(currentMillis - previousMillis > interval)) {
    previousMillis = currentMillis;
    table = loadTable(selected_file, "header");
    println(table.getRowCount() + " total rows in table"); 
      TableRow row = table.getRow(playback_count);
      float x = row.getFloat("x");
      float y = row.getFloat("y");
      float z = row.getFloat("z");
      int g = row.getInt("g");
      float wa = row.getFloat("wa");
      int wr = row.getInt("wr");
      ArmPlayBack(x,y,z,g,wa,wr);
      println(x + "," + y + "," + z + "," + g + "," + wa + "," + wr);
      playback_count++;
      if (playback_count>=table.getRowCount()){
        playback_count = 0;
        PLAYBACK=false;
        println("Playback finished...");
      }
  }
}

public void updateAnimation() {
  long currentMillis = millis();
  if (RECORD&&(currentMillis - previousMillis > interval)) {
    previousMillis = currentMillis;
    saveCoordinates();
  }
}
  static public void main(String[] passedArgs) {
    String[] appletArgs = new String[] { "MiniRobotArm" };
    if (passedArgs != null) {
      PApplet.main(concat(appletArgs, passedArgs));
    } else {
      PApplet.main(appletArgs);
    }
  }
}
