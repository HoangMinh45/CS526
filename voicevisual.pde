import org.openkinect.processing.*;
import ddf.minim.*;
import oscP5.*;

Kinect kinect;
Minim minim;
AudioInput in;
PFont f;
OscP5 oscP5;

float[] depthLookUp = new float[2048];
int dep = 2047;
Particle[] parray = new Particle[640*480];
int skip = 20;
color kolor = color(255, 255, 255);


void setup() {
  
  fullScreen(P3D, 1);
  f = createFont("Carlito", 45, true);
  colorMode(RGB);
  kinect = new Kinect(this);
  kinect.initDepth();
  minim = new Minim(this);
  in = minim.getLineIn(Minim.STEREO, 10);
  oscP5 = new OscP5(this,8000);

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
  rotateY(PI/15);
  
  for(int x = 0; x < kinect.width - skip; x += skip) {
    for (int y = 0; y < kinect.height - skip; y += skip) {
      int offset = x + y*kinect.width;
      int offset1 = x + skip + y*kinect.width;
      int offset2 = x + (y + skip)*kinect.width;
      int offset3 = x + skip + (y + skip)*kinect.width;
      float u = noise(x,y,offset);
     
      int rawDepth = depth[offset];
      PVector v = depthToWorld(x, y, rawDepth);
      
      float factor = 800;
      
      PVector pos = new PVector(v.x*factor, v.y*factor, factor-v.z*factor);
      parray[offset].updateTarget(pos);
      parray[offset].updateXZ(v.x*factor,factor-v.z*factor);
      
      float m = dist(parray[offset].location.x, parray[offset].location.y, parray[offset].location.z, parray[offset1].location.x, parray[offset1].location.y, parray[offset1].location.z);
      float n = dist(parray[offset1].location.x, parray[offset1].location.y, parray[offset1].location.z, parray[offset2].location.x, parray[offset2].location.y, parray[offset2].location.z);
      float o = dist(parray[offset2].location.x, parray[offset2].location.y, parray[offset2].location.z, parray[offset].location.x, parray[offset].location.y, parray[offset].location.z);
      float p = dist(parray[offset3].location.x, parray[offset3].location.y, parray[offset3].location.z, parray[offset2].location.x, parray[offset2].location.y, parray[offset2].location.z);
      
      float sum = 0;
      for(int i = 0; i < in.bufferSize() - 1; i++){
      sum += in.mix.get(i)*20;
      }
      if(sum > 3){
      parray[offset].reFuel(sum);
      }
      
      while(parray[offset].fuel > 0){
        
        float dif = (parray[offset].target.y - parray[offset].location.y);
        if(abs(dif) < 1){
          
          parray[offset].updateY(0);
          
          parray[offset].reFuel(-0.25);
        }else{
          if(abs(dif) < 20){
            parray[offset].updateY(dif*parray[offset].fuel*0.05/abs(dif));
            parray[offset].reFuel(-0.5);
            
          }else{
            parray[offset].updateY(dif*parray[offset].fuel/(10*abs(dif)));
            parray[offset].reFuel(-1);
            
          }
        }
      }
      
      if(parray[offset].fuel < 0){
        parray[offset].fuel = 0;
      }
      
      parray[offset].flowDown(2*u);
      
      
      if(m < 80 && n < 80 && o < 80 && p < 80){
        noFill();
        stroke(kolor);
       
          beginShape(TRIANGLES);
          vertex(parray[offset].location.x, parray[offset].location.y, parray[offset].location.z);
          vertex(parray[offset1].location.x, parray[offset1].location.y, parray[offset1].location.z);
          vertex(parray[offset2].location.x, parray[offset2].location.y, parray[offset2].location.z);
          endShape();
          
          beginShape(TRIANGLES);
          vertex(parray[offset3].location.x, parray[offset3].location.y, parray[offset3].location.z);
          vertex(parray[offset2].location.x, parray[offset2].location.y, parray[offset2].location.z);
          vertex(parray[offset1].location.x, parray[offset1].location.y, parray[offset1].location.z);
          endShape();
        
      }
      
      stroke(255,255,255);
      
        parray[offset].displayPoint();
        
      
    }
  }
  
  saveFrame("frames/####.png");
}

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
  double depth =  depthLookUp[depthValue];
  result.x = (float)((x - cx_d) * depth * fx_d);
  result.y = (float)((y - cy_d) * depth * fy_d);
  result.z = (float)(depth);
  return result;
}

class Particle {
  PVector location;
  PVector target;
  float fuel;
  String txt;
  

  Particle() {
    location = new PVector(500,500,500);
    target = new PVector(500,500,500);
    fuel = 0;
    txt = "";
    
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
    if(location.y < 800){
      location.y += p;
    }
  }
  

  void displayPoint() {
    
    textSize(18);
    text(this.txt, location.x, location.y, location.z);
  }
  
}

void oscEvent(OscMessage msg) {
  for(int x = 0; x < kinect.width - skip; x += skip) {
    for (int y = 0; y < kinect.height - skip; y += skip) {
      int offset = x + y*kinect.width;
      String[] m1 = match(msg.get(1).stringValue(),"happy");
      if(m1 != null){
        kolor = color(0,255,0);
        if(random(8) < 1){
          parray[offset].txt = "happy";
        }else{
          parray[offset].txt = "";
        }
      }
      
      String[] m2 = match(msg.get(1).stringValue(),"sad");
      if(m2 != null){
        kolor = color(255,0,0);
        if(random(8) < 1){
          parray[offset].txt = m2[0];
        }else{
          parray[offset].txt = "";
        }
      }
      
      String[] m3 = match(msg.get(1).stringValue(),"good");
      if(m3 != null){
        kolor = color(0,255,0);
        if(random(8) < 1){
          parray[offset].txt = m3[0];
        }else{
          parray[offset].txt = "";
        }
      }
      
      String[] m4 = match(msg.get(1).stringValue(),"bad");
      if(m4 != null){
        kolor = color(255,0,0);
        if(random(8) < 1){
          parray[offset].txt = m4[0];
        }else{
          parray[offset].txt = "";
        }
      }
      
      String[] m5 = match(msg.get(1).stringValue(),"hopeful");
      if(m5 != null){
        kolor = color(0,255,0);
        if(random(8) < 1){
          parray[offset].txt = m5[0];
        }else{
          parray[offset].txt = "";
        }
      }
      
      String[] m6 = match(msg.get(1).stringValue(),"disappointed");
      if(m6 != null){
        kolor = color(255,0,0);
        if(random(8) < 1){
          parray[offset].txt = m6[0];
        }else{
          parray[offset].txt = "";
        }
      }
      
      String[] m7 = match(msg.get(1).stringValue(),"awesome");
      if(m7 != null){
        kolor = color(0,255,0);
        if(random(8) < 1){
          parray[offset].txt = m7[0];
        }else{
          parray[offset].txt = "";
        }
      }
      
      String[] m8 = match(msg.get(1).stringValue(),"awful");
      if(m8 != null){
        kolor = color(255,0,0);
        if(random(8) < 1){
          parray[offset].txt = m8[0];
        }else{
          parray[offset].txt = "";
        }
      }
      
      String[] m9 = match(msg.get(1).stringValue(),"calm");
      if(m9 != null){
        kolor = color(0,255,0);
        if(random(8) < 1){
          parray[offset].txt = m9[0];
        }else{
          parray[offset].txt = "";
        }
      }
      
      String[] m10 = match(msg.get(1).stringValue(),"angry");
      if(m10 != null){
        kolor = color(255,0,0);
        if(random(8) < 1){
          parray[offset].txt = m10[0];
        }else{
          parray[offset].txt = "";
        }
      }
      
      String[] m11 = match(msg.get(1).stringValue(),"blessed");
      if(m11 != null){
        kolor = color(0,255,0);
        if(random(8) < 1){
          parray[offset].txt = m11[0];
        }else{
          parray[offset].txt = "";
        }
      }
      
      String[] m12 = match(msg.get(1).stringValue(),"unlucky");
      if(m12 != null){
        kolor = color(255,0,0);
        if(random(8) < 1){
          parray[offset].txt = m12[0];
        }else{
          parray[offset].txt = "";
        }
      }
      
      String[] m13 = match(msg.get(1).stringValue(),"optimistic");
      if(m13 != null){
        kolor = color(0,255,0);
        if(random(8) < 1){
          parray[offset].txt = m13[0];
        }else{
          parray[offset].txt = "";
        }
      }
      
      String[] m14 = match(msg.get(1).stringValue(),"pessimistic");
      if(m14 != null){
        kolor = color(255,0,0);
        if(random(8) < 1){
          parray[offset].txt = m14[0];
        }else{
          parray[offset].txt = "";
        }
      }
      
      String[] m15 = match(msg.get(1).stringValue(),"wonderful");
      if(m15 != null){
        kolor = color(0,255,0);
        if(random(8) < 1){
          parray[offset].txt = m15[0];
        }else{
          parray[offset].txt = "";
        }
      }
      
      String[] m16 = match(msg.get(1).stringValue(),"down");
      if(m16 != null){
        kolor = color(255,0,0);
        if(random(8) < 1){
          parray[offset].txt = m16[0];
        }else{
          parray[offset].txt = "";
        }
      }
      
      
    }
  }
}
