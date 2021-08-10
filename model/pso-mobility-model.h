/*
* Author: Benjamin Westburg <benjamin.westburg@temple.edu>
* Affiliation: Temple University - College of Science and Technology
* Date: 2021
*/

#ifndef PSO_MOBILITY_MODEL_H
#define PSO_MOBILITY_MODEL_H

#include <stdint.h>
#include <deque>
#include "ns3/mobility-model.h"
#include "ns3/vector.h"
#include "ns3/constant-velocity-helper.h"
#include "ns3/position-allocator.h"
#include "ns3/ptr.h"
#include "ns3/object.h"
#include "ns3/nstime.h"
#include "ns3/event-id.h"
#include "ns3/box.h"
#include "ns3/random-variable-stream.h"
#include <map>
#include <iterator>
#include <vector>

namespace ns3 {

class PSOMobilityModel : public MobilityModel
{
public:
  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId() const;
  PSOMobilityModel ();
  virtual ~PSOMobilityModel ();
  int DoGetTotalIterations (void);
  void DoSetTotalIterations (int totalIterations);
  double DoGetInertiaWeight (void);
  void DoSetInertiaWeight (const double &inertiaWeight);
  double DoGetIndividualComponent (void);
  void DoSetIndividualComponent (const double &individualComponent);
  double DoGetGroupComponent (void);
  void DoSetGroupComponent (const double &groupComponent);
  double DoGetRandomComponent1 (void);
  void DoSetRandomComponent1 (const double &randomComponent1);
  double DoGetRandomComponent2 (void);
  void DoSetRandomComponent2 (const double &randomComponent2);
  void Update (void);
  Vector DoGetOptimalSolution (void);
  void DoSetOptimalSolution (void);
  Vector DoGetPersonalBestPosition (void);
  void DoSetPersonalBestPosition (void);
  Vector DoGetGroupBestPosition (void);
  void DoSetGroupBestPosition (void);
  double DoGetFitnessValue (void);
  void DoSetFitnessValue (void);
  void EndMobility (void);
  void CalculateVelocity (void);
  static std::map<double,Vector> globalStorage; // double fitness values that correspond to positional vectors for GBEST
  static Vector groupBestPosition;
  static Vector optimalSolution; //3D coordinates for location of objective
  
private:
  double fitnessValue; // fitness values to be cleared each iteration
  int totalIterations;
  int currentIteration;
  double inertiaWeight = 0.9; // make this weight decrease to 0.4 throughout lifetime of sim
                              // 0.5 / totalIterations = decrement of inertia weight per iteration.
  double inertiaWeightDecrement;
  double individualComponent; //c1
  double groupComponent; //c2
  double randomComponent1; //r1
  double randomComponent2; //r2
  ConstantVelocityHelper m_helper;
  Vector personalBestPosition;
  Vector m_position;
  Vector m_velocity;
  //EventId m_event;
  Box m_bounds;
  std::map<double,Vector> personalStorage; // double fitness values that correspond to positional vectors for PBEST
  //static vector <vector<double>> globalBestPosAndFitnessVals;
  virtual Vector DoGetPosition (void) const;
  virtual void DoSetPosition (const Vector &position);
  virtual Vector DoGetVelocity (void) const;
  virtual void DoSetVelocity (const Vector &velocity);
 };
 }
 
 #endif /* PSO_MOBILITY_MODEL_H */

