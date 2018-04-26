// Kinect Physics Example by Amnon Owed (15/09/12)
//edited by Arindam Sen
//updated for openProcessing library and Processing 3 by Erik Nauman 9/16
// import libraries
import org.openkinect.freenect.*;
import org.openkinect.freenect2.*;
import org.openkinect.processing.*;
import org.openkinect.tests.*;

import org.openkinect.processing.*;
import blobDetection.*; // blobs
import toxi.geom.*; // toxiclibs shapes and vectors
import toxi.processing.*; // toxiclibs display
import shiffman.box2d.*; // shiffman's jbox2d helper library
import org.jbox2d.collision.shapes.*; // jbox2d
import org.jbox2d.dynamics.joints.*;
import org.jbox2d.common.*; // jbox2d
import org.jbox2d.dynamics.*; // jbox2d
import processing.opengl.*;

// this is a regular java import so we can use and extend the polygon class (see PolygonBlob)

import java.awt.Polygon;

Kinect kinect;
// declare BlobDetection object
BlobDetection theBlobDetection;

BlobDetection theBlobDetection2;

// ToxiclibsSupport for displaying polygons
ToxiclibsSupport gfx;
// declare custom PolygonBlob object (see class for more info)
PolygonBlob poly;
PolygonBlob2 poly2 = new PolygonBlob2();


// PImage to hold incoming imagery and smaller one for blob detection
PImage blobs;
// the kinect's dimensions to be used later on for calculations
int kinectWidth = 640;
int kinectHeight = 480;
PImage cam = createImage(640, 480, RGB);

//debth variable
int minThresh = 500;
int maxThresh = 600;
// to center and rescale from 640x480 to higher custom resolutions
float reScale;

// background and blob color
color bgColor, blobColor;
// three color palettes (artifact from me storingmany interesting color palettes as strings in an external data file ;-)
String[] palettes = {
  "-1117720,-13683658,-8410437,-9998215,-1849945,-5517090,-4250587,-14178341,-5804972,-3498634", 
  "-67879,-9633503,-8858441,-144382,-4996094,-16604779,-588031", 
  "-1978728,-724510,-15131349,-13932461,-4741770,-9232823,-3195858,-8989771,-2850983,-10314372"
};

color[] colorPalette;


//corot for particles

color bgColor2;

// three color palettes (artifact from me storing many interesting color palettes as strings in an external data file ;-)

String[] palettes2 = {

  "-1117720,-13683658,-8410437,-9998215,-1849945,-5517090,-4250587,-14178341,-5804972,-3498634", 

  "-67879,-9633503,-8858441,-144382,-4996094,-16604779,-588031", 

  "-16711663,-13888933,-9029017,-5213092,-1787063,-11375744,-2167516,-15713402,-5389468,-2064585"

};

// an array called flow of 2250 Particle objects (see Particle class)

Particle[] flow = new Particle[5000];

float globalX, globalY;

int[] screenBuf; 





// the main PBox2D object in which all the physics-based stuff is happening
Box2DProcessing box2d;
// list to hold all the custom shapes (circles, polygons)
ArrayList<CustomShape> polygons = new ArrayList<CustomShape>();

void setup() {
  println("SET UP");
  // it's possible to customize this, for example 1920x1080
  
  
  //customed used to screenbuff
 // screenBuf = new int[1600*900];  // A pixel array to be used later for ShiftBlur
  
  size(1280, 800, P3D);
  kinect = new Kinect(this);
  // mirror the image to be more intuitive
  kinect.enableMirror(true);
  kinect.initDepth();
  // calculate the reScale value
  // currently it's rescaled to fill the complete width (cuts of top-bottom)
  // it's also possible to fill the complete height (leaves empty sides)
  reScale = (float) width / kinectWidth;
  // create a smaller blob image for speed and efficiency
  
  
  blobs = createImage(kinectWidth/3, kinectHeight/3, RGB);
  
  
  
  // initialize blob detection object to the blob image dimensions
  theBlobDetection = new BlobDetection(blobs.width, blobs.height);
  theBlobDetection.setThreshold(0.3);
  
  // initialize ToxiclibsSupport object
  gfx = new ToxiclibsSupport(this);
  // setup box2d, create world, set gravity
  box2d = new Box2DProcessing(this);
  box2d.createWorld();
  box2d.setGravity(0, -30);
  // set random colors (background, blob)
  setRandomColors(1);

//  float gap = kinectWidth / 21;
//  for (int i=0; i<20; i++)
//  {
//    drawString(gap * (i+1), 2, 10);
//  }



//make the flowfield
  setupFlowfield();

}

void drawString(float x, float size, int cards) {

  float gap = kinectHeight/cards;
  // anchor card
  CustomShape s1 = new CustomShape(x, -40, size, BodyType.DYNAMIC);
  polygons.add(s1);

  CustomShape last_shape = s1;
  CustomShape next_shape;
  for (int i=0; i<cards; i++)
  {
    float y = -20 + gap * (i+1);
    next_shape = new CustomShape(x, -20 + gap * (i+1), size, BodyType.DYNAMIC);
    DistanceJointDef jd = new DistanceJointDef();

    Vec2 c1 = last_shape.body.getWorldCenter();
    Vec2 c2 = next_shape.body.getWorldCenter();
    // offset the anchors so the cards hang vertically
    c1.y = c1.y + size / 5;
    c2.y = c2.y - size / 5;
    jd.initialize(last_shape.body, next_shape.body, c1, c2);
    jd.length = box2d.scalarPixelsToWorld(gap - 1);
    box2d.createJoint(jd);
    polygons.add(next_shape);
    last_shape = next_shape;
  }
}

void draw() {
  background(bgColor);
  // update the kinect object
  cam.loadPixels();
  
  blendMode(REPLACE); 
  noStroke();

  // Get the raw depth as array of integers
  int[] depth = kinect.getRawDepth();
  for (int x = 0; x < kinect.width; x++) {
    for (int y = 0; y < kinect.height; y++) {
      int offset = x + y * kinect.width;
      int d = depth[offset];
      
      if(d > minThresh && d < maxThresh){
        
        cam.pixels[offset] = color(255);
        
      }
      else {
        cam.pixels[offset] = color(0);
      }
    }
  }

  cam.updatePixels();
  //image(cam, 0, 0); skip displaying depth image
  // copy the image into the smaller blob image
  blobs.copy(cam, 0, 0, cam.width, cam.height, 0, 0, blobs.width, blobs.height);
  // blur the blob image, otherwise too many blog segments
  blobs.filter(BLUR, 1);
  // detect the blobs
  theBlobDetection.computeBlobs(blobs.pixels);



  poly2 = new PolygonBlob2(); 
  // clear the polygon (original functionality)

  poly2.reset();

  // create the polygon from the blobs (custom functionality, see class)

  poly2.createPolygon();


  drawFlowfield();



   theBlobDetection.computeBlobs(blobs.pixels);
  // initialize a new polygon
  poly = new PolygonBlob();
  // create the polygon from the blobs (custom functionality, see class)
  poly.createPolygon();
  // create the box2d body from the polygon
  poly.createBody();
  // update and draw everything (see method)
  updateAndDrawBox2D();
  // destroy the person's body (important!)
  poly.destroyBody();
  // set the colors randomly every 240th frame
  //setRandomColors(240);



}

//set up the flow field
void setupFlowfield() {

  // set stroke weight (for particle display) to 2.5

  strokeWeight(2.5);

  // initialize all particles in the flow

  for(int i=0; i<flow.length; i++) {

    flow[i] = new Particle(i/10000.0);

  }

  // set all colors randomly now

  setRandomColors2(1);

}






//customed flow field

void drawFlowfield() {

  // center and reScale from Kinect to custom dimensions

  translate(0, (height-kinectHeight*reScale)/2);

  scale(reScale);
  globalX = noise(frameCount * 0.01) * width/2 + width/4;  // set global noise variables that influence the particle flow's movement
  globalY = noise(frameCount * 0.005 + 5) * height;
//  blendMode(REPLACE);

  for (Particle p : flow) {
    p.updateAndDisplay();  // update and display all particles in the flow
  }
  blendMode(ADD);   // Setting blendmode to add since we need the additive effect for the glow
  scale(1/reScale);   // Different images we have used have different sizes, so we need to rescale 
  loadPixels(); // Load what is on the screen
//  shiftBlur3(pixels,screenBuf); // Use the shiftblur method

  // set the colors randomly every 240th frame

  setRandomColors2(240);

}



void updateAndDrawBox2D() {
  // if frameRate is sufficient, add a polygon and a circle with a random radius

  if (frameRate > 10) {
    
    if(frameCount % 4 == 0){
    CustomShape shape1 = new CustomShape(kinectWidth/2, -50, -1, BodyType.DYNAMIC) ;
    //CustomShape shape2 = new CustomShape(kinectWidth/2, -50, random(2.5, 20), BodyType.DYNAMIC);
    polygons.add(shape1);
    //polygons.add(shape2);
    }
    
  }
  // take one step in the box2d physics world
  box2d.step();

  // center and reScale from Kinect to custom dimensions
  translate(0, (height-kinectHeight*reScale)/2);
  scale(reScale);





  // display the person's polygon
  //where we have to change like edding effect
  noStroke();
 // getDepth();








  // display all the shapes  (circles, polygons)
  // go backwards to allow removal of shapes
  for (int i=polygons.size()-1; i>=0; i--) {
    CustomShape cs = polygons.get(i);
    // if the shape is off-screen remove it (see class for more info)

    if (cs.done()) {
      polygons.remove(i);
      // otherwise update (keep shape outside person) and display polygon
    } else {
      cs.update();
      cs.display();
    }
  }
}


// sets the colors every nth frame
void setRandomColors(int nthFrame) {
  if (frameCount % nthFrame == 0) {
    
    // turn a palette into a series of strings
    String[] paletteStrings = split(palettes[int(random(palettes.length))], ",");
    
    // turn strings into colors
    colorPalette = new color[paletteStrings.length];
    for (int i=0; i<paletteStrings.length; i++) {
      colorPalette[i] = int(paletteStrings[i]);
    }
    
    // set background color to first color from palette
    bgColor = colorPalette[0];
    
    // set blob color to second color from palette
    blobColor = colorPalette[1];
    
    // set all shape colors randomly
    for (CustomShape cs : polygons) { 
      cs.col = getRandomColor();
    }
  }
}



//set the color of particles

void setRandomColors2(int nthFrame) {

  if (frameCount % nthFrame == 0) {

    // turn a palette into a series of strings

    String[] paletteStrings = split(palettes2[int(random(palettes.length))], ",");

    // turn strings into colors

    color[] colorPalette = new color[paletteStrings.length];

    for (int i=0; i<paletteStrings.length; i++) {

      colorPalette[i] = int(paletteStrings[i]);

    }

    // set background color to first color from palette

    bgColor2 = colorPalette[0];

    // set all particle colors randomly to color from palette (excluding first aka background color)

    for (int i=0; i<flow.length; i++) {

      flow[i].col = colorPalette[int(random(1, colorPalette.length))];

    }

  }

}










//depth recognization
void  getDepth(){
  
  int drArray[] = {-1,0,0,0,0,0};
  int minus = maxThresh - minThresh ;
  
  for(int i = 1; i <=5; i++){

  drArray[i] = getThresh(minus, 5 , i);
  
  }
  
  
// chang the color depending on the depth
  int[] depth = kinect.getRawDepth();
  for (int x = 0; x < kinect.width; x++) {
    for (int y = 0; y < kinect.height; y++) {
      int offset = x + y * kinect.width;
      int d = depth[offset];
      
      int cal1 = d - minThresh;
      int cal2 = maxThresh - minThresh;
      if(d  > minThresh  && d < maxThresh ){
//      for(int i = 1; i <10; i ++){
//         if(d  > minThresh  + (b* i/10) && d < maxThresh ){
//             color c = color(255 * i)/10);
//             set(x*1280/640 , y* 800/480, c);}

   
   
     color c = color( 255 * (cal2 - cal1)/cal2 , blobColor, 122 * (cal2 - cal1)/cal2, 0);
     set(x*1280/640 , y* 800/480, c);
     set(x*1280/640 + 1 , y* 800/480, c);
     set(x*1280/640 , y* 800/480 +1, c);
     set(x*1280/640 +1, y* 800/480 +1, c);
     
     }
      
    }      
  }
}
  



int getThresh(int num, int div, int mul){
  int a = num / div * mul; 
  return(a);
}


void printPixel(int criteria, int max, int min, int x, int y){
  if(criteria > minThresh && criteria < maxThresh){
           
         color c = color(0, getThresh(255, maxThresh - minThresh ,criteria- minThresh ));
         set(x,y, c);            
       }
}




// returns a random color from the palette (excluding first aka background color)
color getRandomColor() {
  return colorPalette[int(random(1, colorPalette.length))];
}