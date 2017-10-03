/*
 Bullet Continuous Collision Detection and Physics Library Copyright (c) 2007 Erwin Coumans
 Motor Demo
 
 This software is provided 'as-is', without any express or implied warranty.
 In no event will the authors be held liable for any damages arising from the use of this software.
 Permission is granted to anyone to use this software for any purpose,
 including commercial applications, and to alter it and redistribute it freely,
 subject to the following restrictions:
 
 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
 3. This notice may not be removed or altered from any source distribution.
 */


#include <iostream>
#include <vector>
#include "btBulletDynamicsCommon.h"

#include "LinearMath/btIDebugDraw.h"
#include "MotorDemo.h"

#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"

#include "LinearMath/btAlignedObjectArray.h"
class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;

#include "../CommonInterfaces/CommonRigidBodyBase.h"

enum Directions
{
    forward,
    backward
};

class Stage
{
    int  hinges[3] = {};
    bool isHalf;
    Directions direction;
    
public: Stage (int* hinges, bool isHalf, Directions direction)
    {
        for (int i=0; i<3; i++)
            this->hinges[i]=hinges[i];
        this->isHalf = isHalf;
        this->direction = direction;
    }
    
public: Stage (int* hinges, bool isHalf)
    {
        for (int i=0; i<3; i++)
            this->hinges[i]=hinges[i];
        this->isHalf = isHalf;
    }
    
public: int* getHinges()
    {
        return  this->hinges;
    }
    
public: bool getIsHalf()
    {
        return this->isHalf;
    }
    
public: Directions getDirection()
    {
        return this->direction;
    }
    
};

class MotorDemo : public CommonRigidBodyBase
{
    float m_Time;
    float m_fCyclePeriod; // in milliseconds
    float m_fMuscleStrength;
    bool useMCLPSolver = true;
    
    btAlignedObjectArray<class TestRig*> m_rigs;
    
    std::vector<Stage> stages;
    int current_stage;
    std::vector<Stage> stages2;
    bool isComeBack = false;
    
public:
    MotorDemo(struct GUIHelperInterface* helper)
    :CommonRigidBodyBase(helper)
    {
    }
    
    void initPhysics();
    
    void exitPhysics();
    
    virtual ~MotorDemo()
    {
    }
    
    void spawnTestRig(const btVector3& startOffset, bool bFixed);
    
    //	virtual void keyboardCallback(unsigned char key, int x, int y);
    
    void setMotorTargets(btScalar deltaTime);
    
    void resetCamera()
    {
        float dist = 11;
        float pitch = -35;
        float yaw = 52;
        float targetPos[3] = { 0,0.46,0 };
        m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
    }
};


#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2     1.57079632679489661923
#endif

#ifndef M_PI_4
#define M_PI_4     0.785398163397448309616
#endif

#ifndef M_PI_8
#define M_PI_8     0.5 * M_PI_4
#endif


// /LOCAL FUNCTIONS



#define NUM_LEGS 6
#define BODYPART_COUNT 3 * NUM_LEGS + 1
#define JOINT_COUNT BODYPART_COUNT - 1

class TestRig
{
    btDynamicsWorld*	m_ownerWorld;
    btCollisionShape*	m_shapes[BODYPART_COUNT];
    btRigidBody*		m_bodies[BODYPART_COUNT];
    btTypedConstraint*	m_joints[JOINT_COUNT];
    
    btVector3 vUp;
    
    // root global
    btVector3 vRoot;
    btTransform transform;
    
    // Setup rigid bodies global
    btTransform offset;
    
    
    btRigidBody* localCreateRigidBody(btScalar mass, const btTransform& startTransform, btCollisionShape* shape)
    {
        bool isDynamic = (mass != 0.f);
        
        btVector3 localInertia(0, 0, 0);
        if (isDynamic)
            shape->calculateLocalInertia(mass, localInertia);
        
        btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
        btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, shape, localInertia);
        btRigidBody* body = new btRigidBody(rbInfo);
        
        m_ownerWorld->addRigidBody(body);
        
        return body;
    }
    
    
public:
    TestRig(btDynamicsWorld* ownerWorld, const btVector3& positionOffset, bool bFixed)
    : m_ownerWorld(ownerWorld)
    {
        vUp = btVector3(0, 1, 0);
        
        //
        // Setup geometry
        //
        //float fBodySize  = 0.25f;
        float fPreLegLength = 0.25f;
        float fLegLength = 0.45f;
        float fForeLegLength = 0.75f;
        //m_shapes[0] = new btCapsuleShape(btScalar(fBodySize), btScalar(0.10));
        btVector3 fBodySize(1.2, 0.05, 0.8);
        float fAngle = atan2(fBodySize.getZ(), fBodySize.getX());
        float fAlpha = M_PI_4;
        // angle for the leg so that the forleg is vertical
        float fBeta = asin(fPreLegLength*cos(fAlpha) / fLegLength);
        m_shapes[0] = new btBoxShape(fBodySize);
        int i;
        for (i = 0; i<NUM_LEGS; i++)
        {
            m_shapes[1 + 3 * i] = new btCapsuleShape(btScalar(0.10), btScalar(fPreLegLength));
            m_shapes[2 + 3 * i] = new btCapsuleShape(btScalar(0.10), btScalar(fLegLength));
            m_shapes[3 + 3 * i] = new btCapsuleShape(btScalar(0.08), btScalar(fForeLegLength));
        }
        
        //
        // Setup rigid bodies
        //
        float fHeight = 0.5;
        
        offset.setIdentity();
        offset.setOrigin(positionOffset);
        
        // root
        vRoot = btVector3(btScalar(0.), btScalar(fHeight), btScalar(0.));
        
        transform.setIdentity();
        transform.setOrigin(vRoot);
        if (bFixed)
        {
            m_bodies[0] = localCreateRigidBody(btScalar(0.), offset*transform, m_shapes[0]);
        }
        else
        {
            m_bodies[0] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[0]);
        }
        
        // legs
        btVector3 pivotA, axisA, pivotB, axisB;
        
        for (i = 0; i < NUM_LEGS; i++)
        {
            if (i == 0) {
                // setup start position
                btVector3 origin1 = btVector3(btScalar(0.), fHeight, -fBodySize.getZ() - fPreLegLength/2);
                setStartPosition(origin1, true, 1 + 3 * i );
                
                btVector3 origin2 = btVector3(btScalar(0.), fHeight, -fBodySize.getZ() - fPreLegLength - fLegLength);
                setStartPosition(origin2, true, 2 + 3 * i );
                
                btVector3 origin3 = btVector3(btScalar(0.), fHeight - (fForeLegLength / 2), -fBodySize.getZ() - fPreLegLength - fLegLength - fForeLegLength/2);
                setStartPosition(origin3, false, 3 + 3 * i );
                
                // setup constraints
                // hip joints //
                pivotA = btVector3 (btScalar(0.0f), btScalar(0.0f), btScalar(-fBodySize.getZ()));
                pivotB = btVector3 (btScalar(0.0f), btScalar(fPreLegLength / 2), btScalar(0.0f));
                computeConstraints(pivotA, 'y', pivotB, 'y', 'y', btScalar(0), btScalar(0), 0, 1 + 3 * i, 3 * i);
                
                // knee joints //
                pivotA = btVector3 (btScalar(0.0f), btScalar(-fPreLegLength / 2), btScalar(0.0f));
                pivotB = btVector3 (btScalar(0.0f), btScalar(fLegLength / 2), btScalar(0.0f));
                computeConstraints(pivotA, 'y', pivotB, 'y', 'y', btScalar(0), btScalar(0), 1 + 3 * i, 2 + 3 * i, 1 + 3 * i);
                
                // knee joints2 //
                pivotA = btVector3 (btScalar(0.0f), btScalar(-fLegLength / 2), btScalar(0.0f));
                pivotB = btVector3 (btScalar(0.0f), btScalar(fForeLegLength / 2), btScalar(0.0f));
                computeConstraints(pivotA, 'y', pivotB, 'y', 'y', btScalar(0), btScalar(0), 2 + 3 * i, 3 + 3 * i, 2 + 3 * i);
            }
            
            if (i == 1) {
                // setup start position
                btVector3 origin1 = btVector3(-fBodySize.getX() - (fPreLegLength/2)* cos(fAngle), fHeight, -fBodySize.getZ() - sin(fAngle)*fPreLegLength/2);
                setStartPosition(origin1, true, 1 + 3 * i );
                
                btVector3 origin2 = btVector3(-fBodySize.getX() - fPreLegLength*cos(fAngle) - fLegLength*cos(fAngle), fHeight, -fBodySize.getZ() - sin(fAngle)*fPreLegLength - fLegLength*sin(fAngle));
                setStartPosition(origin2, true, 2 + 3 * i );
                
                btVector3 origin3 =(btVector3(-fBodySize.getX() - fPreLegLength*cos(fAngle) - fLegLength*cos(fAngle) -(fForeLegLength/2)*cos(fAngle),
                                              fHeight - (fForeLegLength / 2),
                                              -fBodySize.getZ() - sin(fAngle)*fPreLegLength - fLegLength*sin(fAngle) - sin(fAngle)*(fForeLegLength/2)));
                setStartPosition(origin3, false, 3 + 3 * i );
                
                // setup constraints
                // hip joints //
                pivotA = btVector3(btScalar(-fBodySize.getX()), btScalar(0.0f), btScalar(-fBodySize.getZ()));
                pivotB = btVector3(btScalar(0.0f), btScalar(fPreLegLength / 2), btScalar(0.0f));
                computeConstraints(pivotA, 'y', pivotB, 'y', 'y', btScalar(0), btScalar(0), 0, 1 + 3 * i, 3 * i);
                
                // knee joints //
                pivotA = btVector3(btScalar(0.0f), btScalar(-fPreLegLength / 2), btScalar(0.0f));
                pivotB = btVector3(btScalar(0.0f), btScalar(fLegLength / 2), btScalar(0.0f));
                computeConstraints(pivotA, 'y', pivotB, 'y', 'y', btScalar(0), btScalar(0), 1 + 3 * i, 2 + 3 * i, 1 + 3 * i);
                
                // knee joints2 //
                pivotA = btVector3(btScalar(0.0f), btScalar(-fLegLength / 2), btScalar(0.0f));
                pivotB = btVector3(btScalar(0.0f), btScalar(fForeLegLength / 2), btScalar(0.0f));
                computeConstraints(pivotA, 'y', pivotB, 'y', 'y', btScalar(0), btScalar(0), 2 + 3 * i, 3 + 3 * i, 2 + 3 * i);
            }
            
            if (i == 2) {
                // setup start position
                btVector3 origin1 = btVector3(fBodySize.getX() + (fPreLegLength / 2) * cos(fAngle), fHeight, -fBodySize.getZ() - sin(fAngle)*(fPreLegLength / 2));
                setStartPosition(origin1, true, 1 + 3 * i );
                
                btVector3 origin2 = btVector3( fBodySize.getX() + fPreLegLength*cos(fAngle) + fLegLength*cos(fAngle),
                                              fHeight,
                                              -fBodySize.getZ() -fPreLegLength*sin(fAngle) -fLegLength*sin(fAngle));
                setStartPosition(origin2, true, 2 + 3 * i );
                
                btVector3 origin3 =(btVector3(fBodySize.getX() +fPreLegLength*cos(fAngle) +fLegLength*cos(fAngle) +(fForeLegLength/2)*cos(fAngle),
                                              fHeight - (fForeLegLength / 2),
                                              -fBodySize.getZ() -fPreLegLength*sin(fAngle) -fLegLength*sin(fAngle) -(fForeLegLength/2)*sin(fAngle)));
                setStartPosition(origin3, false, 3 + 3 * i );
                
                // setup constraints
                // hip joints //
                pivotA = btVector3(btScalar(fBodySize.getX()), btScalar(0.0f), btScalar(-fBodySize.getZ()));
                pivotB = btVector3(btScalar(0.0f), btScalar(fPreLegLength / 2), btScalar(0.0f));
                computeConstraints(pivotA, 'y', pivotB, 'y', 'y', btScalar(0), btScalar(0), 0, 1 + 3 * i, 3 * i);
                
                // knee joints //
                pivotA = btVector3(btScalar(0.0f), btScalar(-fPreLegLength / 2), btScalar(0.0f));
                pivotB = btVector3(btScalar(0.0f), btScalar(fLegLength / 2), btScalar(0.0f));
                computeConstraints(pivotA, 'y', pivotB, 'y', 'y', btScalar(0), btScalar(0), 1 + 3 * i, 2 + 3 * i, 1 + 3 * i);
                
                // knee joints2 //
                pivotA = btVector3(btScalar(0.0f), btScalar(-fLegLength / 2), btScalar(0.0f));
                pivotB = btVector3(btScalar(0.0f), btScalar(fForeLegLength / 2), btScalar(0.0f));
                computeConstraints(pivotA, 'y', pivotB, 'y', 'y', btScalar(0), btScalar(0), 2 + 3 * i, 3 + 3 * i, 2 + 3 * i);
            }
            
            if (i == 3) {
                // setup start position
                btVector3 origin1 =  btVector3(btScalar(0.), fHeight, fBodySize.getZ() + (fPreLegLength / 2));
                setStartPosition(origin1, true, 1 + 3 * i );
                
                btVector3 origin2 = btVector3(btScalar(0.), fHeight, fBodySize.getZ() + fPreLegLength + fLegLength);
                setStartPosition(origin2, true, 2 + 3 * i );
                
                btVector3 origin3 =(btVector3(btScalar(0.), fHeight - (fForeLegLength / 2), fBodySize.getZ() + fPreLegLength + fLegLength + (fForeLegLength / 2)));
                setStartPosition(origin3, false, 3 + 3 * i );
                
                // setup constraints
                // hip joints //
                pivotA = btVector3(btScalar(0.0f), btScalar(0.0f), btScalar(fBodySize.getZ()));
                pivotB = btVector3(btScalar(0.0f), btScalar(-fPreLegLength / 2), btScalar(0.0f));
                computeConstraints(pivotA, 'y', pivotB, 'y', 'y', btScalar(0), btScalar(0), 0, 1 + 3 * i, 3 * i);
                
                // knee joints //
                pivotA = btVector3(btScalar(0.0f), btScalar(fPreLegLength / 2), btScalar(0.0f));
                pivotB = btVector3(btScalar(0.0f), btScalar(-fLegLength / 2), btScalar(0.0f));
                computeConstraints(pivotA, 'y', pivotB, 'y', 'y', btScalar(0), btScalar(0), 1 + 3 * i, 2 + 3 * i, 1 + 3 * i);
                
                // knee joints2 //
                pivotA = btVector3(btScalar(0.0f), btScalar(fLegLength / 2), btScalar(0.0f));
                pivotB = btVector3(btScalar(0.0f), btScalar(-fForeLegLength / 2), btScalar(0.0f));
                computeConstraints(pivotA, 'y', pivotB, 'y', 'y', btScalar(0), btScalar(0), 2 + 3 * i, 3 + 3 * i, 2 + 3 * i);
            }
            
            if (i == 4) {
                // setup start position
                btVector3 origin1 =  btVector3(-fBodySize.getX() - (fPreLegLength / 2)* cos(fAngle), fHeight, fBodySize.getZ() + sin(fAngle)*(fPreLegLength / 2));
                setStartPosition(origin1, true, 1 + 3 * i );
                
                btVector3 origin2 = btVector3( -fBodySize.getX() - fPreLegLength*cos(fAngle) - fLegLength*cos(fAngle),
                                              fHeight,
                                              fBodySize.getZ() + sin(fAngle)*fPreLegLength + sin(fAngle)*fLegLength);
                setStartPosition(origin2, true, 2 + 3 * i );
                
                btVector3 origin3 =(btVector3(-fBodySize.getX() - fPreLegLength*cos(fAngle) -fLegLength*cos(fAngle) -(fForeLegLength / 2)*cos(fAngle),
                                              fHeight - (fForeLegLength / 2),
                                              fBodySize.getZ() + fPreLegLength*sin(fAngle) + fLegLength*sin(fAngle) + (fForeLegLength / 2)*sin(fAngle)));
                setStartPosition(origin3, false, 3 + 3 * i );
                
                // setup constraints
                // hip joints //
                pivotA = btVector3(btScalar(-fBodySize.getX()), btScalar(0.0f), btScalar(fBodySize.getZ()));
                pivotB = btVector3(btScalar(0.0f), btScalar(-fPreLegLength / 2), btScalar(0.0f));
                computeConstraints(pivotA, 'y', pivotB, 'y', 'y', btScalar(0), btScalar(0), 0, 1 + 3 * i, 3 * i);
                
                // knee joints //
                pivotA = btVector3(btScalar(0.0f), btScalar(fPreLegLength / 2), btScalar(0.0f));
                pivotB = btVector3(btScalar(0.0f), btScalar(-fLegLength / 2), btScalar(0.0f));
                computeConstraints(pivotA, 'y', pivotB, 'y', 'y', btScalar(0), btScalar(0), 1 + 3 * i, 2 + 3 * i, 1 + 3 * i);
                
                // knee joints2 //
                pivotA = btVector3(btScalar(0.0f), btScalar(fLegLength / 2), btScalar(0.0f));
                pivotB = btVector3(btScalar(0.0f), btScalar(-fForeLegLength / 2), btScalar(0.0f));
                computeConstraints(pivotA, 'y', pivotB, 'y', 'y', btScalar(0), btScalar(0), 2 + 3 * i, 3 + 3 * i, 2 + 3 * i);
            }
            
            if (i == 5) {
                // setup start position
                btVector3 origin1 =  btVector3( fBodySize.getX() + (fPreLegLength / 2)* cos(fAngle), fHeight, fBodySize.getZ() + sin(fAngle)*(fPreLegLength / 2));
                setStartPosition(origin1, true, 1 + 3 * i );
                
                btVector3 origin2 = btVector3(btVector3(
                                                        fBodySize.getX() + fPreLegLength*cos(fAngle) + fLegLength*cos(fAngle),
                                                        fHeight,
                                                        fBodySize.getZ() + sin(fAngle)*fPreLegLength + sin(fAngle)*fLegLength));
                setStartPosition(origin2, true, 2 + 3 * i );
                
                btVector3 origin3 =(btVector3(fBodySize.getX() + fPreLegLength*cos(fAngle) + fLegLength*cos(fAngle) + (fForeLegLength / 2)*cos(fAngle),
                                              fHeight - (fForeLegLength / 2),
                                              fBodySize.getZ() + fPreLegLength*sin(fAngle) + fLegLength*sin(fAngle) + (fForeLegLength / 2)*sin(fAngle)));
                setStartPosition(origin3, false, 3 + 3 * i );
                
                // setup constraints
                // hip joints //
                pivotA = btVector3(btScalar(fBodySize.getX()), btScalar(0.0f), btScalar(fBodySize.getZ()));
                pivotB = btVector3(btScalar(0.0f), btScalar(-fPreLegLength / 2), btScalar(0.0f));
                computeConstraints(pivotA, 'y', pivotB, 'y', 'y', btScalar(0), btScalar(0), 0, 1 + 3 * i, 3 * i);
                
                // knee joints //
                pivotA = btVector3(btScalar(0.0f), btScalar(fPreLegLength / 2), btScalar(0.0f));
                pivotB = btVector3(btScalar(0.0f), btScalar(-fLegLength / 2), btScalar(0.0f));
                computeConstraints(pivotA, 'y', pivotB, 'y', 'y', btScalar(0), btScalar(0), 1 + 3 * i, 2 + 3 * i, 1 + 3 * i);
                
                // knee joints2 //
                pivotA = btVector3(btScalar(0.0f), btScalar(fLegLength / 2), btScalar(0.0f));
                pivotB = btVector3(btScalar(0.0f), btScalar(-fForeLegLength / 2), btScalar(0.0f));
                computeConstraints(pivotA, 'y', pivotB, 'y', 'y', btScalar(0), btScalar(0), 2 + 3 * i, 3 + 3 * i, 2 + 3 * i);
            }
            
        }
        
        // Setup some damping on the m_bodies
        for (i = 0; i < BODYPART_COUNT; ++i)
        {
            m_bodies[i]->setDamping(0.05, 0.85);
            m_bodies[i]->setDeactivationTime(0.8);
            m_bodies[i]->setSleepingThresholds(0.5f, 0.5f);//1.6, 2.5
        }
        
    }
    
    
    virtual	~TestRig()
    {
        int i;
        
        // Remove all constraints
        for (i = 0; i < JOINT_COUNT; ++i)
        {
            m_ownerWorld->removeConstraint(m_joints[i]);
            delete m_joints[i]; m_joints[i] = 0;
        }
        
        // Remove all bodies and shapes
        for (i = 0; i < BODYPART_COUNT; ++i)
        {
            m_ownerWorld->removeRigidBody(m_bodies[i]);
            
            delete m_bodies[i]->getMotionState();
            
            delete m_bodies[i]; m_bodies[i] = 0;
            delete m_shapes[i]; m_shapes[i] = 0;
        }
    }
    
    btTypedConstraint** GetJoints() { return &m_joints[0]; }
    
    
    void computeConstraints (btVector3& pivotA, char axisAName, btVector3& pivotB, char axisBName, char jointAxis, float limitFrom,
                             float limitTo, int bodyFrom, int bodyTo, int joint )
    {
        btVector3 axisA = setAxis(axisAName);
        btVector3 axisB = setAxis(axisBName);
        
        btHingeConstraint* hingeC = new btHingeConstraint(*m_bodies[bodyFrom], *m_bodies[bodyTo], pivotA, pivotB, axisA, axisB);
        btVector3 vectorA = setAxis(jointAxis);
        
        hingeC->setAxis(vectorA);
        hingeC->setLimit(limitFrom, limitTo);
        
        m_joints[joint] = hingeC;
        m_ownerWorld->addConstraint(m_joints[joint], true);
    }
    
    btVector3 setAxis (char axis)
    {
        btVector3 vector;
        
        if (axis=='x') vector = btVector3(btScalar(1.0f), btScalar(0.0f), btScalar(0.0f));
        else if (axis=='y') vector = btVector3(btScalar(0.0f), btScalar(1.0f), btScalar(0.0f));
        else vector = btVector3(btScalar(0.0f), btScalar(0.0f), btScalar(1.0f));
        
        return vector;
    }
    
    void setStartPosition( btVector3& origin, bool isRotate, int shape)
    {
        transform.setIdentity();
        transform.setOrigin(origin);
        
        if (isRotate)
        {
            btVector3 vToBone = (origin - vRoot).normalize(); //
            btVector3 vAxis = vToBone.cross(vUp); //
            transform.setRotation(btQuaternion(vAxis, M_PI_2)); //
        }
        
        m_bodies[shape] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[shape]);
    }
    
    
};



void motorPreTickCallback(btDynamicsWorld *world, btScalar timeStep)
{
    MotorDemo* motorDemo = (MotorDemo*)world->getWorldUserInfo();
    
    motorDemo->setMotorTargets(timeStep);
    
}



void MotorDemo::initPhysics()
{
    m_guiHelper->setUpAxis(1);
    
    // Setup the basic world
    
    m_Time = 0;
    m_fCyclePeriod = 2000.f; // in milliseconds
    
    //	m_fMuscleStrength = 0.05f;
    // new SIMD solver for joints clips accumulated impulse, so the new limits for the motor
    // should be (numberOfsolverIterations * oldLimits)
    // currently solver uses 10 iterations, so:
    m_fMuscleStrength = 0.5f;
    
    
    m_collisionConfiguration = new btDefaultCollisionConfiguration();
    
    m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
    
    btVector3 worldAabbMin(-10000, -10000, -10000);
    btVector3 worldAabbMax(10000, 10000, 10000);
    m_broadphase = new btAxisSweep3(worldAabbMin, worldAabbMax);
    
    if (useMCLPSolver)
    {
        btDantzigSolver* mlcp = new btDantzigSolver();
        //btSolveProjectedGaussSeidel* mlcp = new btSolveProjectedGaussSeidel;
        btMLCPSolver* sol = new btMLCPSolver(mlcp);
        m_solver = sol;
    } else
    {
        m_solver = new btSequentialImpulseConstraintSolver();
    }
    
    m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);
    
    m_dynamicsWorld->setInternalTickCallback(motorPreTickCallback, this, true);
    m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
    
    
    // Setup a big ground box
    {
        btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(200.), btScalar(10.), btScalar(200.)));
        m_collisionShapes.push_back(groundShape);
        btTransform groundTransform;
        groundTransform.setIdentity();
        groundTransform.setOrigin(btVector3(0, -10, 0));
        createRigidBody(btScalar(0.), groundTransform, groundShape);
    }
    
    // Spawn one ragdoll
    btVector3 startOffset(1, 0.5, 0);
    spawnTestRig(startOffset, false);
    //    startOffset.setValue(-2,0.5,0);
    //    spawnTestRig(startOffset, true);
    
    m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
    
    int stage1_arr[] = {7,10,4};
    Stage* stage1 = new Stage( stage1_arr, true, Directions::forward);
    stages.push_back(*stage1);
    
    int stage2_arr[] = {8,11,5};
    Stage* stage2 = new Stage( stage2_arr, true, Directions::forward);
    stages.push_back(*stage2);
    
    int stage3_arr[] = {6,9,3};
    Stage* stage3 = new Stage( stage3_arr, false);
    stages.push_back(*stage3);
    
    int stage4_arr[] = {8,11,5};
    Stage* stage4 = new Stage( stage4_arr, true, Directions::backward);
    stages.push_back(*stage4);
    
    int stage5_arr[] = {7,10,4};
    Stage* stage5 = new Stage( stage5_arr, true, Directions::backward);
    stages.push_back(*stage5);
    
    int stage6_arr[] = {16,1,13};
    Stage* stage6 = new Stage( stage6_arr, true, Directions::forward);
    stages.push_back(*stage6);
    
    int stage7_arr[] = {17,2,14};
    Stage* stage7 = new Stage( stage7_arr, true, Directions::forward);
    stages.push_back(*stage7);
    
    int stage8_arr[] = {15,0,12};
    Stage* stage8 = new Stage( stage8_arr, false);
    stages.push_back(*stage8);
    
    int stage9_arr[] = {17,2,14};
    Stage* stage9 = new Stage( stage9_arr, true, Directions::backward);
    stages.push_back(*stage9);
    
    int stage10_arr[] = {16,1,13};
    Stage* stage10 = new Stage( stage10_arr, true, Directions::backward);
    stages.push_back(*stage10);
    
    
    
    
    int stage1_arra[] = {6};
    Stage* stage1a = new Stage( stage1_arra, false);
    stages2.push_back(*stage1a);
    
    int stage2_arra[] = {15};
    Stage* stage2a = new Stage( stage2_arra, false);
    stages2.push_back(*stage2a);
    
    int stage3_arra[] = {0};
    Stage* stage3a = new Stage( stage3_arra, false);
    stages2.push_back(*stage3a);
    
    int stage4_arra[] = {9};
    Stage* stage4a = new Stage( stage4_arra, false);
    stages2.push_back(*stage4a);
    
    int stage5_arra[] = {3};
    Stage* stage5a = new Stage( stage5_arra, false);
    stages2.push_back(*stage5a);
    
    int stage6_arra[] = {12};
    Stage* stage6a = new Stage( stage6_arra, false);
    stages2.push_back(*stage6a);
    
    current_stage = 0;
    
    
}


void MotorDemo::spawnTestRig(const btVector3& startOffset, bool bFixed)
{
    TestRig* rig = new TestRig(m_dynamicsWorld, startOffset, bFixed);
    m_rigs.push_back(rig);
}

void	PreStep()
{
    
}




void MotorDemo::setMotorTargets(btScalar deltaTime)
{
    float ms = deltaTime*1000000.;
    float minFPS = 1000000.f / 60.f;
    if (ms > minFPS)
        ms = minFPS;
    
    m_Time += ms;
    
    //
    // set per-frame sinusoidal position targets using angular motor (hacky?)
    //
    //    float currentAngles [1] = {};
    //    float angleLimits [1] = {};
    
    if (current_stage >= 6) current_stage = 0;
    float current_angle;
    float up_limit;
    float low_limit;
    
    for (int r = 0; r<m_rigs.size(); r++)
    {
        // for (int i = 0; i <1 /*3 *NUM_LEGS */; i++)
        // {
        int current_joint = stages2[current_stage].getHinges()[0];
        
        //  std::cout<<"Current joint "<<current_joint<<std::endl;
        btHingeConstraint* hingeC = static_cast<btHingeConstraint*>(m_rigs[r]->GetJoints()[current_joint]);
        if (current_joint == 6 || current_joint == 0 || current_joint == 3)
        {
            hingeC->setLimit(-0.45, 0.45);
        }
        else
        {
            hingeC->setLimit(0.45, -0.45);
        }
        
        btScalar fCurAngle = hingeC->getHingeAngle();
        btScalar fTargetPercent = (int(m_Time / 1000) % int(m_fCyclePeriod)) / m_fCyclePeriod;
        btScalar fTargetAngle = 0.5 * (1 + sin(2 * M_PI * fTargetPercent)); // change
        btScalar fTargetLimitAngle = hingeC->getLowerLimit() + fTargetAngle * (hingeC->getUpperLimit() - hingeC->getLowerLimit());
        btScalar fAngleError = fTargetLimitAngle - fCurAngle;
        btScalar fDesiredAngularVel = 1000000.f * fAngleError / ms; //change
        hingeC->enableAngularMotor(true, fDesiredAngularVel*10, m_fMuscleStrength);
        
        current_angle = hingeC->getHingeAngle();
        low_limit = hingeC->getLowerLimit();
        up_limit = hingeC->getUpperLimit();
        
        //        std::cout<<"Current stage "<<current_stage<<std::endl;
        //        std::cout<<"Current upper limit "<<hingeC->getUpperLimit()<<std::endl;
        //        std::cout<<"Current lower limit "<<hingeC->getLowerLimit()<<std::endl;
        //        std::cout<<"Current angle: "<<hingeC->getHingeAngle()<<std::endl<<std::endl;
        //  }
        
        
        if (up_limit>0)
        {
            if (current_angle >= up_limit-abs (up_limit/10) && !isComeBack)
            {
                isComeBack = true;
                std::cout<<"isComeBack = true"<<std::endl;
                std::cout<<"Current stage "<<current_stage<<std::endl<<std::endl;
            }
            
            
            if (current_angle <= low_limit+abs(low_limit/10) && isComeBack)
            {
                isComeBack = false;
                std::cout<<"isComeBack = false"<<std::endl;
                current_stage++;
                hingeC->setLimit(0, 0);
                std::cout<<"Current stage "<<current_stage<<std::endl<<std::endl;
            }
        }
        
        
        else
        {
            if (current_angle >= low_limit-abs (low_limit/10) && !isComeBack)
            {
                isComeBack = true;
                std::cout<<"isComeBack = true"<<std::endl;
                std::cout<<"Current stage "<<current_stage<<std::endl<<std::endl;
            }
            
            if (current_angle <= up_limit+abs (up_limit/10) && isComeBack)
            {
                isComeBack = false;
                std::cout<<"isComeBack = false"<<std::endl;
                current_stage++;
                hingeC->setLimit(0, 0);
                std::cout<<"Current stage "<<current_stage<<std::endl<<std::endl;
            }
        }
        
    }
    
    
    
}

#if 0
void MotorDemo::keyboardCallback(unsigned char key, int x, int y)
{
    switch (key)
    {
        case '+': case '=':
            m_fCyclePeriod /= 1.1f;
            if (m_fCyclePeriod < 1.f)
                m_fCyclePeriod = 1.f;
            break;
        case '-': case '_':
            m_fCyclePeriod *= 1.1f;
            break;
        case '[':
            m_fMuscleStrength /= 1.1f;
            break;
        case ']':
            m_fMuscleStrength *= 1.1f;
            break;
        default:
            DemoApplication::keyboardCallback(key, x, y);
    }
}
#endif



void MotorDemo::exitPhysics()
{
    
    int i;
    
    for (i = 0; i<m_rigs.size(); i++)
    {
        TestRig* rig = m_rigs[i];
        delete rig;
    }
    
    //cleanup in the reverse order of creation/initialization
    
    //remove the rigidbodies from the dynamics world and delete them
    
    for (i = m_dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
    {
        btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
        btRigidBody* body = btRigidBody::upcast(obj);
        if (body && body->getMotionState())
        {
            delete body->getMotionState();
        }
        m_dynamicsWorld->removeCollisionObject(obj);
        delete obj;
    }
    
    //delete collision shapes
    for (int j = 0; j<m_collisionShapes.size(); j++)
    {
        btCollisionShape* shape = m_collisionShapes[j];
        delete shape;
    }
    
    //delete dynamics world
    delete m_dynamicsWorld;
    
    //delete solver
    delete m_solver;
    
    //delete broadphase
    delete m_broadphase;
    
    //delete dispatcher
    delete m_dispatcher;
    
    delete m_collisionConfiguration;
}


class CommonExampleInterface*    MotorControlCreateFunc(struct CommonExampleOptions& options)
{
    return new MotorDemo(options.m_guiHelper);
}
