#include "../src/linked_finger_servo.hpp"
#include "../src/contact_evidence.hpp"
#include <cstdlib>
#include <iostream>
#include <limits>
void check(bool ok) { if (!ok) { std::cerr << "Servo regression failed\n"; std::exit(1); } }
int main() {
  using fr3_inspection::LinkedFingerServo;
  LinkedFingerServo servo;
  auto force = servo.step(.002, {.0175, .0175}, {0, 0}, .001);
  check(std::abs(servo.reference-.017494) < 1e-12);
  check(force[0] == force[1] && force[0] < 0);
  // A blocked 12 mm shaft never causes accumulating or unbounded drive force.
  for (int i=0; i<10000; ++i) {
    force=servo.step(.002, {.00705,.00705}, {0,0}, .001);
    check(std::abs(force[0]) <= 4 && std::abs(force[1]) <= 4);
  }
  check(force[0] == -4 && force[1] == -4);
  // A lagging finger gets opposite synchronization correction.
  servo.reference=.01;
  force=servo.step(.01, {.0098,.0102}, {0,0}, .001);
  check(force[0]>0 && force[1]<0);
  force=servo.step(std::numeric_limits<double>::quiet_NaN(), {.01,.01}, {0,0}, .001);
  check(force[0]==0 && force[1]==0);
  // Free motion integrates force; neither finger position is assigned to a command.
  servo=LinkedFingerServo{};
  std::array<double,2> q{.0175,.0175}, v{0,0};
  for (int i=0; i<5000; ++i) {
    force=servo.step(.00705,q,v,.001);
    for (int j=0;j<2;++j) { v[j]+=(force[j]-15*v[j])/.1*.001; q[j]+=v[j]*.001; }
  }
  check(std::abs(q[0]-.00705)<.0001 && std::abs(q[1]-q[0])<1e-9);
  fr3_inspection::ContactEvidence contacts;
  contacts.sample(1, {.0001, -1});
  check(!contacts.ready(1));
  contacts.sample(1.01, {.0002,.0003});
  check(!contacts.ready(1.02));
  for (int i=2; i<=13; ++i) contacts.sample(1+i*.01, {.0002,.0003});
  check(contacts.ready(1.14));
  check(!contacts.ready(1.3));
  contacts.sample(1.14, {.004,.0001});
  check(!contacts.ready(1.14));
  contacts.sample(.1, {-1,-1});
  check(!contacts.ready(.1));
  std::cout << "Linked finger servo tests passed\n";
}
