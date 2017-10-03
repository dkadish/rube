import processing.serial.*; //import the Serial library
Serial myPort;  //the Serial port object
String val;
int nPosSamples, nSpdSamples;

import grafica.*;
GPlot posPlot, spdPlot;
GPointsArray posPoints = new GPointsArray(0);
GPointsArray spdPoints = new GPointsArray(0);

import java.util.*;
import java.util.concurrent.ArrayBlockingQueue;
ArrayBlockingQueue posQueue = new ArrayBlockingQueue(1000);
ArrayBlockingQueue spdQueue = new ArrayBlockingQueue(1000);

void setup() {
  size(600, 600); //make our canvas 200 x 200 pixels big
  //  initialize your serial port and set the baud rate to 9600
  println(Serial.list());
  myPort = new Serial(this, Serial.list()[3], 9600);
  myPort.bufferUntil('\n');
  nPosSamples = 0;
  nSpdSamples = 0;
  
  plotSetup();
}

void plotSetup(){
    // Create the first plot
  posPlot = new GPlot(this);
  posPlot.setPos(0, 0);
  posPlot.setMar(60, 70, 40, 70);
  posPlot.setDim(450, 450);
  posPlot.setAxesOffset(4);
  posPlot.setTicksLength(4);
 
  // Create the second plot with the same dimensions
  spdPlot = new GPlot(this);
  spdPlot.setPos(posPlot.getPos());
  spdPlot.setMar(posPlot.getMar());
  spdPlot.setDim(posPlot.getDim());
  spdPlot.setAxesOffset(4);
  spdPlot.setTicksLength(4);
 
  // Set the points, the title and the axis labels
  posPlot.setPoints(posPoints);
  posPlot.setTitleText("Position");
  posPlot.getYAxis().setAxisLabelText("Position (Steps)");
  posPlot.getXAxis().setAxisLabelText("Time (milliseconds)");
 
  spdPlot.setPoints(spdPoints);
  spdPlot.getRightAxis().setAxisLabelText("Speed (Steps/millis)");
  //spdPlot.getYAxis().setLog(true);
  spdPlot.setYLim(-0.6,1.0);
 
  // Make the right axis of the second plot visible
  spdPlot.getRightAxis().setDrawTickLabels(true);
 
  // Activate the panning
  //posPlot.activatePanning();
  //spdPlot.activatePanning();
}

void draw() {
  //we can leave the draw method empty, 
  //because all our programming happens in the serialEvent (see below)
  
  while(posQueue.peek() != null){
    posPoints.add((float)nPosSamples, (float)posQueue.poll());
    nPosSamples++;
  }
  while(spdQueue.peek() != null){
    spdPoints.add((float)nSpdSamples, (float)spdQueue.poll());
    nSpdSamples++;
  }
  
  while(posPoints.getNPoints() > 1000){
    posPoints.remove(0);
  }
  while(spdPoints.getNPoints() > 1000){
    spdPoints.remove(0);
  }
    
  posPlot.setPoints(posPoints);
  spdPlot.setPoints(spdPoints);
  
  background(255);
 
  // Draw the first plot
  posPlot.beginDraw();
  posPlot.drawBox();
  posPlot.drawXAxis();
  posPlot.drawYAxis();
  posPlot.drawTitle();
  //posPlot.drawPoints();
  posPlot.drawLines();
  posPlot.endDraw();
 
  // Draw only the right axis
  spdPlot.beginDraw();
  spdPlot.drawRightAxis();
  //spdPlot.drawPoints();
  spdPlot.drawLines();
  spdPlot.endDraw();
}

void serialEvent( Serial myPort ) {
  //put the incoming data into a String - 
  //the '\n' is our end delimiter indicating the end of a complete packet
  val = myPort.readStringUntil('\n');
  //make sure our data isn't empty before continuing
  if (val != null) {
    //trim whitespace and formatting characters (like carriage return)
    val = trim(val);
    //println(val);
    String[] movementData = split(val, ',');
    int pos = int(movementData[0]);
    float speed = float(movementData[2]);
    println((float)pos, speed);
    
    posQueue.add((float)pos);
    spdQueue.add(speed);
  }
  
}