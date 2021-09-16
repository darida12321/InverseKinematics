
class Arm{
  float RADIOUS = 15; // The radious of the displayed circles

  int parts;          // The number of parts it has
  PVector[] positions; // The position of the joints
  float[] lengths;     // the length of the arms

  // Constructor
  Arm(PVector start_pos, float[] _lengths){
    parts = _lengths.length;
    positions = new PVector[parts+1];
    lengths = new float[parts];

    positions[0] = start_pos;
    for(int i = 0; i < parts; i++){
      lengths[i] = _lengths[i];
      positions[i+1] = positions[i].copy().add(new PVector(lengths[i], 0));
    }
  }

  // Inverse kinematics
  void reachFor(PVector pos){
    float old_dist = 0;
    float new_dist = pos.copy().sub(positions[parts]).mag();

    do{
      inverseKinematicsStep(pos);
      old_dist = new_dist;
      new_dist = pos.copy().sub(positions[parts]).mag();
    }while(old_dist - new_dist > 0.001);
  }

  void inverseKinematicsStep(PVector pos){
    PVector start_pos = positions[0].copy();

    positions[parts] = pos.copy();
    for(int i = parts-1; i >= 0; i--){
      PVector normal = positions[i].copy().sub(positions[i+1]).normalize();
      positions[i] = positions[i+1].copy().add(normal.mult(lengths[i]));
    }

    positions[0] = start_pos;
    for(int i = 0; i < parts; i++){
      PVector normal = positions[i+1].copy().sub(positions[i]).normalize();
      positions[i+1] = positions[i].copy().add(normal.mult(lengths[i]));
    }
  }

  // Display arm
  void display(){
    // Display first point
    stroke(255, 0, 0);
    fill(255, 0, 0);
    ellipse(positions[0].x, positions[0].y, RADIOUS, RADIOUS);
    // Display rest of the points
    stroke(255);
    fill(255);
    for(int i = 0; i < parts; i++){
      ellipse(positions[i+1].x, positions[i+1].y, RADIOUS, RADIOUS);
      line(positions[i].x, positions[i].y, positions[i+1].x, positions[i+1].y);
    }
  }
}

float[] lengths = {60, 80, 40, 100, 30, 50, 50};
Arm arm = new Arm(new PVector(500, 500), lengths);

void setup(){
  size(1000, 1000);
}

void draw(){
  //println(frameRate);
  background(0);
  arm.reachFor(new PVector(mouseX, mouseY));
  arm.display();
}
