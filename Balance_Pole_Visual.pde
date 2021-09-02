import g4p_controls.*;
import processing.serial.*;
import grafica.*;
import java.awt.Font;
import java.util.concurrent.locks.ReentrantLock;
import java.util.concurrent.TimeUnit;


GTextField txf1;
Serial myPort; 
GLabel PIDSettings;
GTextArea serialData;
String readings="";
int capture = 500;
int on =0;
ReentrantLock lock = new ReentrantLock();
boolean ed = false;
boolean updateTerm= false;

//Graphing Stuff
GPlot plot, plot2;
GPointsArray points, pwmPoints;
void setup() {
  background(.5);
  size(3500, 1750);
  
  readings="";
  //Serial Setup
  try {
    myPort = new Serial(this, "COM8", 115200);
  }
  catch(Exception e) {
   System.exit(0);
  }
  myPort.clear();
  myPort.bufferUntil(10);
  //Serial Label
  
  serialData = new GTextArea(this,2650,100,800,1625,GConstants.SCROLLBARS_VERTICAL_ONLY);
  serialData.setFont(new Font("Monospaced", Font.PLAIN, 30));
  serialData.setTextEditEnabled(false);
  //PID Label
  PIDSettings = new GLabel(this, 50, 1600, 2500, 50, "PID Settings");
  PIDSettings.setFont(new Font("Monospaced", Font.PLAIN, 30));

  //Command Box
  txf1 = new GTextField(this, 50, 1650, 2550, 75);
  txf1.setPromptText("Send Command");
  txf1.setFont(new Font("Monospaced", Font.PLAIN, 30));
  textFont(createFont("Arial", 20), 20);


  //Graph Setup
  int labelSize = 25;
  plot = new GPlot(this, 50, 100, 2500, 1450);
  plot.setMar(75, 75, 75, 75);
  plot.getYAxis().setFontSize(labelSize);
  plot.getYAxis().getAxisLabel().setFontSize(labelSize);
  plot.getXAxis().setFontSize(labelSize);
  plot.getXAxis().getAxisLabel().setFontSize(labelSize);
  plot.getTitle().setFontSize(30);
  plot.setTitleText("Output Data");
  plot.getXAxis().setAxisLabelText("Time");
  plot.getYAxis().setAxisLabelText("Angle");
  plot.setYLim(0, 180);
  plot.getYAxis().setTicksSeparation(30.0);
  points = new GPointsArray(capture);
  pwmPoints = new GPointsArray(capture);
  for (int i =0; i<capture; i++) {
    points.add(new GPoint(0, 0));
    pwmPoints.add(new GPoint(0, 0));
  }
  plot.setPoints(points);
  
  //plot 2

  plot2 = new GPlot(this);
  plot2.setPos(plot.getPos());
  plot2.setMar(plot.getMar());
  plot2.setDim(plot.getDim());
  plot2.setAxesOffset(4);
  plot2.setTicksLength(4);
  plot2.setYLim(-255, 255);
  plot2.getRightAxis().getAxisLabel().setFontSize(labelSize);
  plot2.getRightAxis().setFontSize(labelSize);
  plot2.getRightAxis().setAxisLabelText("Motor PWM");
  plot2.getRightAxis().setTicksSeparation(51);
  // Make the right axis of the second plot visible
  plot2.getRightAxis().setDrawTickLabels(true);
  plot2.addLayer("pwmlayer", pwmPoints);
  plot2.getLayer("pwmlayer").setPointColor(color(29, 82, 121));
  frameRate(60);
}


public void draw() {
  try {
    if (!lock.tryLock(0, TimeUnit.MILLISECONDS)) {
      return;
    }
  }
  catch(Exception e ) {
    e.printStackTrace();
    return;
  }

  if(updateTerm){
    updateTerm=false;
    serialData.setText(readings);

  }
    
  background(209);
  try {
    plot.defaultDraw();
    plot2.beginDraw();
    plot2.drawPoints();
    plot2.drawRightAxis();
    plot2.endDraw();
  }
  catch(Exception e) {
    e.printStackTrace();
    throw e;
  }
  finally {
    lock.unlock();
  }
}

public void handleTextEvents(GEditableTextControl textcontrol, GEvent event) {
  if (txf1 == textcontrol && event == GEvent.ENTERED) {
    myPort.write(new String(txf1.getText()));
    textcontrol.setText("");
  }
}

void serialEvent(Serial p) {
  try {
    if (!lock.tryLock(2, TimeUnit.MILLISECONDS)) {
      println("lock skip");
      return;
    }
  }
  catch(Exception e ) {
    e.printStackTrace();
    return;
  }
  
  try {
    String inBuffer = p.readStringUntil(10);
    if (inBuffer==null||inBuffer.length()<3) {
      return;
    }
    if (inBuffer.contains("Current Settings|")) {
      PIDSettings.setText(inBuffer.substring(17));
    } else if (inBuffer.contains("Data:")) {
      plot.removePoint(0);
      plot.addPoint(millis(), Float.parseFloat(inBuffer.substring(5)));
    } else if (inBuffer.contains("Output:")) {
      if (inBuffer.contains("STOP") ){
        plot2.removePoint(0, "pwmlayer");
        plot2.addPoint(millis(), 0, "", "pwmlayer");
      } else {
        plot2.removePoint(0, "pwmlayer");
        plot2.addPoint(millis(), Float.parseFloat(inBuffer.substring(7)), "", "pwmlayer");
      }
    } else {
      if(readings.length()>2000){
        readings=readings.substring(1000);
      }
      readings+=inBuffer;
      updateTerm=true;
    }
    on++;
    if (on==110) {
      myPort.clear();
      on=0;
    }
  }
  catch(Exception e) {
    println("Brokeey");
    e.printStackTrace();
    System.exit(0);
  }
  finally {
    lock.unlock();
  }
}
