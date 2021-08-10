/*
* Author: Benjamin Westburg <benjamin.westburg@temple.edu>
* Affiliation: Temple University - College of Science and Technology
* Date: 2021
*/

#include "pso-mobility-model.h"
#include <limits>
#include "ns3/abort.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"
#include "ns3/log.h"
#include "ns3/boolean.h"
#include "ns3/config.h"
#include "ns3/test.h"
#include <cmath>
#include <vector>
#include "ns3/random-variable-stream.h"
#include "ns3/double.h"

namespace ns3 {

Vector PSOMobilityModel::optimalSolution;
Vector PSOMobilityModel::groupBestPosition;
std::map<double,Vector> PSOMobilityModel::globalStorage;

NS_LOG_COMPONENT_DEFINE ("PSOMobilityModel");
NS_OBJECT_ENSURE_REGISTERED (PSOMobilityModel);

TypeId
ns3::PSOMobilityModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::PSOMobilityModel")
    .SetParent<MobilityModel>()
    .SetGroupName ("Mobility")
    .AddConstructor<PSOMobilityModel>() 		   		    
   ;
   return tid;
}
TypeId ns3::PSOMobilityModel::GetInstanceTypeId() const
{
  return PSOMobilityModel::GetTypeId();
}

ns3::PSOMobilityModel::PSOMobilityModel ()
{
  totalIterations = 100;
  individualComponent = 2.0;
  groupComponent = 2.0;
  randomComponent1 = 0.5;
  randomComponent2 = 0.5;
  optimalSolution.x = 55.0;
  optimalSolution.y = 25.0;
  optimalSolution.z = 15.0;
  inertiaWeightDecrement = 0.5/totalIterations;
  double VeloRNGMin = -10;
  double VeloRNGMax = 10;
  Ptr<UniformRandomVariable> randNumGen = CreateObject<UniformRandomVariable> ();
  randNumGen->SetAttribute ("Min", DoubleValue (VeloRNGMin));
  randNumGen->SetAttribute ("Max", DoubleValue (VeloRNGMax));
  m_velocity.x = randNumGen->GetValue ();
  m_velocity.y = randNumGen->GetValue ();
  m_velocity.z = randNumGen->GetValue ();
  for (int i = 0; i <= 200; i++) {
    Simulator::Schedule (Seconds (i), &PSOMobilityModel::Update, this);
  }
  //m_helper.Unpause ();
}

ns3::PSOMobilityModel::~PSOMobilityModel ()
{
}

void
PSOMobilityModel::Update (void)
{
  if (DoGetPosition() == DoGetOptimalSolution()) {
  	EndMobility();
  }
  DoSetFitnessValue();
  DoSetPersonalBestPosition();
  DoSetGroupBestPosition();
  CalculateVelocity();
  DoSetPosition(DoGetPosition());
  std::cout << "pos:" << DoGetPosition() << std::endl;
  std::cout << "fit val:" << DoGetFitnessValue() << std::endl;
  std::cout << "opt val:" << DoGetOptimalSolution() << std::endl;
  std::cout << "group best pos:" << DoGetGroupBestPosition() << std::endl;
  //Simulator::Schedule (Seconds (1), &PSOMobilityModel::Update, this);
}

inline Vector
PSOMobilityModel::DoGetPosition (void) const
{
  return Vector (m_position.x, m_position.y, m_position.z);
}

void
PSOMobilityModel::DoSetPosition (const Vector &position)
{
  // X sub i+1 = X sub i + V sub i+1
  Vector newPosition = position + m_velocity;
  m_position = newPosition;
  m_helper.SetPosition (m_position);
  //m_event.Cancel ();
  //m_event = Simulator::ScheduleNow (&PSOMobilityModel::Start, this);
}

inline Vector
PSOMobilityModel::DoGetVelocity (void) const
{
  return Vector (m_velocity.x, m_velocity.y, m_velocity.z);
}

void
PSOMobilityModel::CalculateVelocity (void)
{
  //V sub i+1 = w * V sub i + c1r1 (PBEST sub i - current position) + c2r2 (GBEST sub i - current position)
  
  //inertia component calculation
  double inertiaComponentVectorX = m_velocity.x * inertiaWeight;
  double inertiaComponentVectorY = m_velocity.y * inertiaWeight;
  double inertiaComponentVectorZ = m_velocity.z * inertiaWeight;
  
  //personal component calculation
  double personalComponentVectorX = personalBestPosition.x - m_position.x;
  double personalComponentVectorY = personalBestPosition.y - m_position.y;
  double personalComponentVectorZ = personalBestPosition.z - m_position.z;
  double personalComponentScalar = individualComponent * randomComponent1;
  personalComponentVectorX = personalComponentVectorX * personalComponentScalar;
  personalComponentVectorY = personalComponentVectorY * personalComponentScalar;
  personalComponentVectorZ = personalComponentVectorZ * personalComponentScalar;
  
  //group component calculation
  Vector GBP = DoGetGroupBestPosition();
  double groupComponentVectorX = GBP.x - m_position.x;
  double groupComponentVectorY = GBP.y - m_position.y;
  double groupComponentVectorZ = GBP.z - m_position.z;
  double groupComponentScalar = groupComponent * randomComponent2;
  groupComponentVectorX = groupComponentVectorX * groupComponentScalar;
  groupComponentVectorY = groupComponentVectorY * groupComponentScalar;
  groupComponentVectorZ = groupComponentVectorZ * groupComponentScalar;
  
  //new velocity vector calculation
  double newVelocityVectorX = inertiaComponentVectorX + personalComponentVectorX + groupComponentVectorX;
  double newVelocityVectorY = inertiaComponentVectorY + personalComponentVectorY + groupComponentVectorY;
  double newVelocityVectorZ = inertiaComponentVectorZ + personalComponentVectorZ + groupComponentVectorZ;
  
  Vector newVelocity (newVelocityVectorX,newVelocityVectorY,newVelocityVectorZ);
  DoSetVelocity(newVelocity);
}

void
PSOMobilityModel::DoSetVelocity (const Vector &velocity)
{
  Vector newVelocity (velocity.x,velocity.y,velocity.z);
  m_velocity = newVelocity;
  m_helper.SetVelocity (m_velocity);
  //m_event.Cancel ();
  //m_event = Simulator::ScheduleNow (&PSOMobilityModel::Start, this);
}

Vector
PSOMobilityModel::DoGetOptimalSolution (void)
{
  return optimalSolution;
}

void 
PSOMobilityModel::DoSetOptimalSolution (void)
{
}

Vector
PSOMobilityModel::DoGetPersonalBestPosition (void)
{
  return personalBestPosition;
}

void
PSOMobilityModel::DoSetPersonalBestPosition (void)
{
  // find largest double entry in map, and set corresponding positional vector as PBEST (personalBestPosition).
  // calculation of PBEST
  std::map<double, Vector>::iterator currentEntryP; // iteration over personal storage
  std::pair<double, Vector> entryWithMinValueP = std::make_pair(5000000.0, Vector(0.0,0.0,0.0)); // for reference to find max
  for (currentEntryP = personalStorage.begin();
       currentEntryP != personalStorage.end();
       ++currentEntryP) {
       if (currentEntryP->first
           < entryWithMinValueP.first) {
           entryWithMinValueP
               = std::make_pair(
                   currentEntryP->first,
                   currentEntryP->second);
       }
   }
   Vector entryWithMinValuePVector = entryWithMinValueP.second;
   personalBestPosition = entryWithMinValuePVector;
}

Vector
PSOMobilityModel::DoGetGroupBestPosition (void)
{
  return groupBestPosition;
}

void 
PSOMobilityModel::DoSetGroupBestPosition (void)
{
  // find largest double entry in map, and set corresponding positional vector as GBEST (groupBestPosition).
  // calculation of GBEST 
  std::map<double, Vector>::iterator currentEntryG; // iteration over personal storage
  std::pair<double, Vector> entryWithMinValueG = std::make_pair(5000000.0, Vector(0.0,0.0,0.0)); // for reference to find max
  for (currentEntryG = globalStorage.begin();
       currentEntryG != globalStorage.end();
       ++currentEntryG) {
       if (currentEntryG->first
           < entryWithMinValueG.first) {
           entryWithMinValueG
               = std::make_pair(
                   currentEntryG->first,
                   currentEntryG->second);
       }
   }
   Vector entryWithMinValueGVector = entryWithMinValueG.second;
   groupBestPosition = entryWithMinValueGVector;
}

double
PSOMobilityModel::DoGetFitnessValue (void)
{
  return fitnessValue;
}

void
PSOMobilityModel::DoSetFitnessValue (void) 
{
  // fitness(optimization) function: 10 * (x - optimal_solution sub x)^2 + 20 * (y - optimal_solution sub y)^2 + 30 * (z -
  // optimal_solution sub z)^2
  // global optimum (solution) is x sub 1 = 55, x sub 2 = 25, x sub 3 = 15
  // goal is to attain minimum fitness value.
  double x = m_position.x;
  double y = m_position.y;
  double z = m_position.z;
  double optSolX = optimalSolution.x;
  double optSolY = optimalSolution.y;
  double optSolZ = optimalSolution.z;
  double base1 = x - optSolX;
  double base2 = y - optSolY;
  double base3 = z - optSolZ;
  double power = 2.0;
  fitnessValue = 10 * pow(base1,power) + 20 * pow(base2,power) + 30 * pow(base3,power);
  personalStorage.insert({ fitnessValue, m_position });
  globalStorage.insert({ fitnessValue, m_position });
}

void
PSOMobilityModel::EndMobility (void)
{
}

}


    		   

