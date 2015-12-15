import org.openkinect.processing.*;
import ddf.minim.*;

Kinect kinect;
Minim minim;
AudioInput in;

float[] depthLookUp = new float[2048];
int dep = 2047;
Particle[] parray = new Particle[640*480];
int skip = 15;


void setup() {
  size(800, 600, P3D);
  //fullScreen(P3D, 1);
  kinect = new Kinect(this);
  kinect.initDepth();
  minim = new Minim(this);
  in = minim.getLineIn(Minim.STEREO, 10);

  for (int i = 0; i < depthLookUp.length; i++) {
    depthLookUp[i] = rawDepthToMeters(i);
  }
  
  for (int m = 0; m < kinect.width; m += skip) {
    for (int n = 0; n < kinect.height; n += skip) {
      int off = m + n*kinect.width;
      parray[off] = new Particle();
    }
  }
}

void draw() {
  background(0);
  int[] depth = kinect.getRawDepth();
  
  translate(width/2, height/2, -50);
  rotateY(PI/20);
  
  for(int x = 0; x < kinect.width; x += skip) {
    for (int y = 0; y < kinect.height; y += skip) {
      int offset = x + y*kinect.width;
      float u = noise(x,y,depth[offset]);

      // Convert kinect data to world xyz coordinate
      int rawDepth = depth[offset];
      PVector v = depthToWorld(x, y, rawDepth);
      // Scale up by 800 times
      float factor = 800;
      PVector pos = new PVector(v.x*factor, v.y*factor, factor-v.z*factor);
      parray[offset].updateTarget(pos);
      parray[offset].updateXZ(v.x*factor,factor-v.z*factor);
      
      int sum = 0;
      for(int i = 0; i < in.bufferSize() - 1; i++){
        sum += round(in.mix.get(i)*5)%10;
      }
  
      parray[offset].reFuel(sum);
  
      while(parray[offset].fuel > 0){
        
        float dif = (parray[offset].target.y - parray[offset].location.y);
        if(abs(dif) < 1){
          parray[offset].updateY(0);
          parray[offset].display(parray[offset].location, rawDepth, parray[offset].fuel);
          parray[offset].reFuel(-1);
        }else{
          if(abs(dif) < 30){
            parray[offset].updateY(dif*parray[offset].fuel/abs(dif));
            parray[offset].display(parray[offset].location, rawDepth, 5);
            parray[offset].reFuel(-1);
          }else{
            parray[offset].updateY(dif*parray[offset].fuel/abs(dif));
            parray[offset].display(parray[offset].location, rawDepth, 5);
            parray[offset].reFuel(-1);
          }
        }
      }
      
      if(parray[offset].fuel < 0){
        parray[offset].fuel = 0;
      }
      
      parray[offset].flowDown(2*u);
      parray[offset].display(parray[offset].location, rawDepth, 5);
    }
  }
  saveFrame("frames/####.png");
}

// These functions come from: http://graphics.stanford.edu/~mdfisher/Kinect.html
float rawDepthToMeters(int depthValue) {
  if (depthValue < 2047) {
    return (float)(1.0 / ((double)(depthValue) * -0.0030711016 + 3.3309495161));
  }
  return 0.0f;
}

PVector depthToWorld(int x, int y, int depthValue) {

  final double fx_d = 1.0 / 5.9421434211923247e+02;
  final double fy_d = 1.0 / 5.9104053696870778e+02;
  final double cx_d = 3.3930780975300314e+02;
  final double cy_d = 2.4273913761751615e+02;

  PVector result = new PVector();
  double depth =  depthLookUp[depthValue];//rawDepthToMeters(depthValue);
  result.x = (float)((x - cx_d) * depth * fx_d);
  result.y = (float)((y - cy_d) * depth * fy_d);
  result.z = (float)(depth);
  return result;
}

class Particle {
  PVector location;
  PVector target;
  float fuel;

  Particle() {
    location = new PVector(500,500,500);
    target = new PVector(500,500,500);
    fuel = 0;
  }
  
  void reFuel(float gas){
    fuel += gas;
  }
  
  void updateTarget(PVector k){
    target = k;
  }
  
  void updateXZ(float a, float b){
    location.x = a;
    location.z = b;
  }

  void updateY(float change) {
    location.y += change;
  }
  void flowDown(float p){
    if(location.y < 1000){
      location.y += p;
    }
  }

  void display(PVector v, int d, float r) {
    pushMatrix();
    translate(v.x, v.y, v.z);
    noFill();
    colorMode(HSB, 360, 100, 100);
    stroke(map(d*r, 0, 2047*9, 0, 360), 100, 100);
    ellipse(0, 0, r, r);
    popMatrix();
  }
}
