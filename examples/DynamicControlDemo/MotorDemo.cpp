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

struct Limits
{
    btScalar lowLimit;
    btScalar upLimit;
    
public: Limits (btScalar lowLimit, btScalar upLimit)
    {
        this->lowLimit = lowLimit;
        this->upLimit = upLimit;
    }
    
public: Limits () {}
    
};

class Stage
{
    int  hinges_1[3] = {};
    Limits limits_1[3];
    Directions direction_1;
    
    int  hinges_2[3] = {};
    Limits limits_2[3];
    Directions direction_2;
    
public: Stage (int* hinges_1, Limits* limits_1, Directions direction_1, int* hinges_2, Limits* limits_2, Directions direction_2)
    {
        for (int i=0; i<3; i++)
        {
            this->hinges_1[i]=hinges_1[i];
            this->limits_1[i]=limits_1[i];
            this->hinges_2[i]=hinges_2[i];
            this->limits_2[i]=limits_2[i];
        }
        
        this->direction_1 = direction_1;
        this->direction_2 = direction_2;
    }
    
public: int* getHinges_1()
    {
        return  this->hinges_1;
    }
    
public: int* getHinges_2()
    {
        return  this->hinges_2;
    }
    
public: Directions getDirection_1()
    {
        return this->direction_1;
    }
    
public: Directions getDirection_2()
    {
        return this->direction_2;
    }
    
public: Limits* getLimits_1()
    {
        return  this->limits_1;
    }
    
public: Limits* getLimits_2()
    {
        return  this->limits_2;
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
    bool isfinished_1 = false;
    bool isfinished_2 = false;
    float time_passed = 0;
    
    
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
    
    //    virtual void keyboardCallback(unsigned char key, int x, int y);
    
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
    btDynamicsWorld*    m_ownerWorld;
    btCollisionShape*    m_shapes[BODYPART_COUNT];
    btRigidBody*        m_bodies[BODYPART_COUNT];
    btTypedConstraint*    m_joints[JOINT_COUNT];
    
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
        btVector3 fBodySize(1.2, 0.05, 0.6);
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
        btVector3 pivotA0, axisA0, pivotB0, axisB0, pivotA1, axisA1, pivotB1, axisB1, pivotA2, axisA2, pivotB2, axisB2;
        btVector3 pivotA3, axisA3, pivotB3, axisB3, pivotA4, axisA4, pivotB4, axisB4, pivotA5, axisA5, pivotB5, axisB5;
        btVector3 pivotA6, axisA6, pivotB6, axisB6, pivotA7, axisA7, pivotB7, axisB7, pivotA8, axisA8, pivotB8, axisB8;
        
        btVector3 pivotA9, axisA9, pivotB9, axisB9, pivotA10, axisA10, pivotB10, axisB10, pivotA11, axisA11, pivotB11, axisB11;
        btVector3 pivotA12, axisA12, pivotB12, axisB12, pivotA13, axisA13, pivotB13, axisB13, pivotA14, axisA14, pivotB14, axisB14;
        btVector3 pivotA15, axisA15, pivotB15, axisB15, pivotA16, axisA16, pivotB16, axisB16, pivotA17, axisA17, pivotB17, axisB17;
        btVector3 pivotA, axisA, pivotB, axisB;
        for (i = 0; i < NUM_LEGS; i++)
        {
            float NewfAngle = 2 * M_PI * i / 6;//NUM_LEGS;
            float fSin = sin(NewfAngle);
            float fCos = cos(NewfAngle);
            
            if (i == 0) {
                // setup start position
                btVector3 origin1 = btVector3(btScalar(0.), fHeight, -fBodySize.getZ() - fPreLegLength);
                transform.setIdentity();
                transform.setOrigin(origin1);
                
                btVector3 vToBone1 = (origin1 - vRoot).normalize();
                btVector3 vAxis1 = vToBone1.cross(vUp); //
                transform.setRotation(btQuaternion(vAxis1, M_PI_2)); //
                m_bodies[1 + 3 * i] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[1 + 3 * i]);
                //setStartPosition(origin1, true, 1 + 3 * i );
                /********************************************************************************************************/
                btVector3 origin2 = btVector3(btScalar(0.), fHeight, -fBodySize.getZ() - fPreLegLength - fLegLength);
                transform.setIdentity();
                transform.setOrigin(origin2);
                
                btVector3 vToBone2 = (origin2 - vRoot).normalize();
                btVector3 vAxis2 = vToBone2.cross(vUp); //
                transform.setRotation(btQuaternion(vAxis2, M_PI_2)); //
                m_bodies[2 + 3 * i] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[2 + 3 * i]);
                //setStartPosition(origin2, true, 2 + 3 * i );
                /********************************************************************************************************/
                btVector3 origin3 = btVector3(btScalar(0.), fHeight - (fForeLegLength / 2), -fBodySize.getZ() - fPreLegLength - fLegLength - fForeLegLength/2);
                transform.setIdentity();
                transform.setOrigin(origin3);
                
                btVector3 vToBone3 = (origin3 - vRoot).normalize(); ////for 1,2,4,5 vToBone.cross(vUp) = vAxis
                btVector3 vAxis3 = vToBone3.cross(vUp); //
                //transform.setRotation(btQuaternion(vAxis, M_PI_2)); //
                m_bodies[3 + 3 * i] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[3 + 3 * i]);
                //setStartPosition(origin3, false, 3 + 3 * i );
                /********************************************************************************************************/
                // setup constraints
                // hip joints //
                pivotA0 = btVector3 (btScalar(0.0f), btScalar(0.0f), btScalar(-fBodySize.getZ()));//fHight
                pivotB0 = btVector3 (btScalar(0.0f), btScalar(fPreLegLength), btScalar(0.0f));//placing joint
                btVector3 axisA0 = btVector3(btScalar(0.0f), btScalar(-1.0f), btScalar(0.0f));//setAxis('y');
                btVector3 axisB0 = btVector3(btScalar(1.0f), btScalar(0.0f), btScalar(0.0f));//setAxis('x');
                
                btHingeConstraint* hingeC0 = new btHingeConstraint(*m_bodies[3 * i], *m_bodies[1 + 3 * i], pivotA0, pivotB0, axisA0, axisB0);
                hingeC0->setLimit(-M_PI_2, -M_PI_2); //setLimit(-2*M_PI/3,-M_PI_4);//setLimit(-M_PI_2, -M_PI_2);
                
                m_joints[3 * i] = hingeC0;
                m_ownerWorld->addConstraint(m_joints[3 * i], true);
                //computeConstraints(pivotA, 'y', pivotB, 'x', 'y', btScalar(0.), btScalar(0.), 0, 1 + 3 * i, 3 * i);
                /********************************************************************************************************/
                // knee joints //
                pivotA1 = btVector3 (btScalar(0.0f), btScalar(-fPreLegLength), btScalar(0.0f));
                pivotB1 = btVector3 (btScalar(0.0f), btScalar(fLegLength), btScalar(0.0f));
                btVector3 axisA1 = btVector3(btScalar(0.0f), btScalar(0.0f), btScalar(-1.0f));//setAxis('y');
                btVector3 axisB1 = btVector3(btScalar(1.0f), btScalar(0.0f), btScalar(0.0f));//setAxis('y');
                
                btHingeConstraint* hingeC1 = new btHingeConstraint(*m_bodies[1 + 3 * i], *m_bodies[2 + 3 * i], pivotA1, pivotB1, axisA1, axisB1);
                hingeC1->setLimit(-M_PI_8, -M_PI_8);//btScalar(0.0f), btScalar(0.0f));M_PI_8
                m_joints[1 + 3 * i] = hingeC1;
                m_ownerWorld->addConstraint(m_joints[1 + 3 * i], true);
                
                //computeConstraints(pivotA, 'y', pivotB, 'y', 'y', btScalar(0), btScalar(0), 1 + 3 * i, 2 + 3 * i, 1 + 3 * i);
                /********************************************************************************************************/
                // knee joints2 //
                pivotA2 = btVector3 (btScalar(0.0f), btScalar(-fLegLength/2), btScalar(0.0f));
                pivotB2 = btVector3 (btScalar(0.0f), btScalar(fForeLegLength/2), btScalar(0.0f));
                btVector3 axisA2 = btVector3(btScalar(-1.0f), btScalar(0.0f), btScalar(0.0f));//setAxis('y');
                btVector3 axisB2 = btVector3(btScalar(0.0f), btScalar(0.0f), btScalar(1.0f));//setAxis('y');
                
                btHingeConstraint* hingeC2 = new btHingeConstraint(*m_bodies[2 + 3 * i], *m_bodies[3 + 3 * i], pivotA2, pivotB2, axisA2, axisB2);
                hingeC2->setLimit(-2*M_PI_4, -2*M_PI_4);//-M_PI_4
                
                m_joints[2 + 3 * i] = hingeC2;
                m_ownerWorld->addConstraint(m_joints[2 + 3 * i], true);
                //computeConstraints(pivotA, 'y', pivotB, 'y', 'y', btScalar(0), btScalar(0), 2 + 3 * i, 3 + 3 * i, 2 + 3 * i);
            }
            
            if (i == 1) {
                // setup start position
                btVector3 origin11 = btVector3((-fBodySize.getX() - (fPreLegLength)* cos(fAngle)), fHeight, (-fBodySize.getZ() -(fPreLegLength) * sin(fAngle)));
                transform.setIdentity();
                transform.setOrigin(origin11);
                
                btVector3 vToBone11 = (origin11 - vRoot).normalize(); ////for 1,2,4,5 vToBone.cross(vUp) = vAxis
                btVector3 vAxis11 = vToBone11.cross(vUp); //
                transform.setRotation(btQuaternion(vAxis11, M_PI_2)); //
                m_bodies[1 + 3 * i] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[1 + 3 * i]);
                //setStartPosition(origin1, true, 1 + 3 * i );
                /********************************************************************************************************/
                btVector3 origin21 = btVector3((-fBodySize.getX() - (fPreLegLength)*cos(fAngle) - (fLegLength)*cos(fAngle)), fHeight, (-fBodySize.getZ() - (fPreLegLength)*sin(fAngle) - (fLegLength)*sin(fAngle)));
                transform.setIdentity();
                transform.setOrigin(origin21);
                
                btVector3 vToBone21 = (origin21 - vRoot).normalize(); ////for 1,2,4,5 vToBone.cross(vUp) = vAxis
                btVector3 vAxis21 = vToBone21.cross(vUp); //
                transform.setRotation(btQuaternion(vAxis21, M_PI_2)); //
                m_bodies[2 + 3 * i] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[2 + 3 * i]);
                //setStartPosition(origin22, true, 2 + 3 * i );
                /********************************************************************************************************/
                btVector3 origin31 = (btVector3(-fBodySize.getX() - fPreLegLength*cos(fAngle) - fLegLength*cos(fAngle) - (fForeLegLength / 2)*cos(fAngle),
                                                fHeight - (fForeLegLength / 2),
                                                -fBodySize.getZ() - sin(fAngle)*fPreLegLength - fLegLength*sin(fAngle) - sin(fAngle)*(fForeLegLength / 2)));
                transform.setIdentity();
                transform.setOrigin(origin31);
                
                btVector3 vToBone31 = (origin31 - vRoot).normalize(); ////for 1,2,4,5 vToBone.cross(vUp) = vAxis
                btVector3 vAxis31 = vToBone31.cross(vUp); //
                //transform.setRotation(btQuaternion(vAxis22, M_PI_2)); //
                m_bodies[3 + 3 * i] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[3 + 3 * i]);
                //setStartPosition(origin32, false, 3 + 3 * i );
                /********************************************************************************************************/
                // setup constraints
                // hip joints //
                pivotA3 = btVector3(btScalar(-fBodySize.getX()), btScalar(0.0f), btScalar(-fBodySize.getZ()));//cos and sin
                pivotB3 = btVector3(btScalar(0.0f), btScalar(fPreLegLength), btScalar(0.0f));///2
                btVector3 axisA3 = btVector3(btScalar(0.0f), btScalar(-1.0f), btScalar(0.0f));//-1
                btVector3 axisB3 = btVector3(btScalar(1.0f), btScalar(0.0f), btScalar(0.0f));
                
                btHingeConstraint* hingeC3 = new btHingeConstraint(*m_bodies[0], *m_bodies[1 + 3 * i], pivotA3, pivotB3, axisA3, axisB3);
                hingeC3->setLimit(-M_PI_4, -M_PI_4);//setLimit(-2 * M_PI / 3, -M_PI_4);//setLimit(-M_PI_2,-M_PI_2); or -M_PI_2,-M_PI_2;
                
                m_joints[3 * i] = hingeC3;
                m_ownerWorld->addConstraint(m_joints[3 * i], true);
                //computeConstraints(pivotA, 'y', pivotB, 'y', 'y', btScalar(0), btScalar(0), 0, 1 + 3 * i, 3 * i);
                /********************************************************************************************************/
                // knee joints //
                pivotA4 = btVector3(btScalar(0.0f), btScalar(-fPreLegLength), btScalar(0.0f));
                pivotB4 = btVector3(btScalar(0.0f), btScalar(fLegLength), btScalar(0.0f));
                btVector3 axisA4 = btVector3(btScalar(0.0f), btScalar(0.0f), btScalar(-1.0f));//setAxis('y');
                btVector3 axisB4 = btVector3(btScalar(1.0f), btScalar(0.0f), btScalar(0.0f));//setAxis('x');
                
                btHingeConstraint* hingeC4 = new btHingeConstraint(*m_bodies[1 + 3 * i], *m_bodies[2 + 3 * i], pivotA4, pivotB4, axisA4, axisB4);
                hingeC4->setLimit(-M_PI_8, -M_PI_8);//M_PI_8
                
                m_joints[1 + 3 * i] = hingeC4;
                m_ownerWorld->addConstraint(m_joints[1+3 * i], true);
                //computeConstraints(pivotA, 'y', pivotB, 'y', 'y', btScalar(0), btScalar(0), 1 + 3 * i, 2 + 3 * i, 1 + 3 * i);
                /********************************************************************************************************/
                // knee joints2 //
                pivotA5 = btVector3(btScalar(0.0f), btScalar(-fLegLength / 2), btScalar(0.0f));
                pivotB5 = btVector3(btScalar(0.0f), btScalar(fForeLegLength / 2), btScalar(0.0f));
                btVector3 axisA5 = btVector3(btScalar(-1.0f), btScalar(0.0f), btScalar(0.0f));//setAxis('y');1.0.0
                btVector3 axisB5 = btVector3(btScalar(0.0f), btScalar(0.0f), btScalar(1.0f));//setAxis('x');0.0.-1
                
                btHingeConstraint* hingeC5 = new btHingeConstraint(*m_bodies[2 + 3 * i], *m_bodies[3 + 3 * i], pivotA5, pivotB5, axisA5, axisB5);
                hingeC5->setLimit(-2 * M_PI_4, -2*M_PI_4);//2*M_PI_4
                
                m_joints[2+3 * i] = hingeC5;
                m_ownerWorld->addConstraint(m_joints[2 + 3 * i], true);
                //computeConstraints(pivotA, 'y', pivotB, 'y', 'y', btScalar(0), btScalar(0), 2 + 3 * i, 3 + 3 * i, 2 + 3 * i);
            }
            
            if (i == 2) {
                // setup start position
                btVector3 origin12 = btVector3(fBodySize.getX() + (fPreLegLength / 2)* cos(fAngle), fHeight, -fBodySize.getZ() - sin(fAngle)*fPreLegLength / 2);
                transform.setIdentity();
                transform.setOrigin(origin12);
                
                btVector3 vToBone12 = (origin12 - vRoot).normalize(); ////for 1,2,4,5 vToBone.cross(vUp) = vAxis
                btVector3 vAxis12 = vToBone12.cross(vUp); //
                transform.setRotation(btQuaternion(vAxis12, M_PI_2)); //
                m_bodies[1 + 3 * i] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[1 + 3 * i]);
                //setStartPosition(origin1, true, 1 + 3 * i );
                /********************************************************************************************************/
                btVector3 origin22 = btVector3(fBodySize.getX() + fPreLegLength*cos(fAngle) + fLegLength*cos(fAngle), fHeight, -fBodySize.getZ() - sin(fAngle)*fPreLegLength - fLegLength*sin(fAngle));
                transform.setIdentity();
                transform.setOrigin(origin22);
                
                btVector3 vToBone22 = (origin22 - vRoot).normalize(); ////for 1,2,4,5 vToBone.cross(vUp) = vAxis
                btVector3 vAxis22 = vToBone22.cross(vUp); //
                transform.setRotation(btQuaternion(vAxis22, M_PI_2)); //
                m_bodies[2 + 3 * i] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[2 + 3 * i]);
                //setStartPosition(origin2, true, 2 + 3 * i );
                /********************************************************************************************************/
                btVector3 origin32 = (btVector3(fBodySize.getX() + fPreLegLength*cos(fAngle) + fLegLength*cos(fAngle) + (fForeLegLength / 2)*cos(fAngle),
                                                fHeight - (fForeLegLength / 2),
                                                -fBodySize.getZ() - sin(fAngle)*fPreLegLength - fLegLength*sin(fAngle) - sin(fAngle)*(fForeLegLength / 2)));
                transform.setIdentity();
                transform.setOrigin(origin32);
                
                btVector3 vToBone32 = (origin32 - vRoot).normalize(); ////for 1,2,4,5 vToBone.cross(vUp) = vAxis
                btVector3 vAxis32 = vToBone32.cross(vUp); //
                //transform.setRotation(btQuaternion(vAxis32, M_PI_2)); //
                m_bodies[3 + 3 * i] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[3 + 3 * i]);
                // setStartPosition(origin3, false, 3 + 3 * i );
                /********************************************************************************************************/
                // setup constraints
                // hip joints //
                pivotA6 = btVector3(btScalar(fBodySize.getX()), btScalar(0.0f), btScalar(-fBodySize.getZ()));
                pivotB6 = btVector3(btScalar(0.0f), btScalar(fPreLegLength), btScalar(0.0f));//2
                btVector3 axisA6 = btVector3(btScalar(0.0f), btScalar(-1.0f), btScalar(0.0f));//setAxis('y');1
                btVector3 axisB6 = btVector3(btScalar(1.0f), btScalar(0.0f), btScalar(0.0f));//setAxis('x');
                
                btHingeConstraint* hingeC6 = new btHingeConstraint(*m_bodies[0], *m_bodies[1 + 3 * i], pivotA6, pivotB6, axisA6, axisB6);
                hingeC6->setLimit(-M_PI_2, -M_PI_2);//setLimit(-2 * M_PI / 3,-M_PI_4);//setLimit(3 * M_PI / 2, 3 * M_PI / 2);//-M_PI_2
                //hingeC3->setLimit(-M_PI_2, -M_PI_2);
                m_joints[3 * i] = hingeC6;
                m_ownerWorld->addConstraint(m_joints[3 * i], true);
                //computeConstraints(pivotA, 'y', pivotB, 'y', 'y', btScalar(0), btScalar(0), 0, 1 + 3 * i, 3 * i);
                
                // knee joints //
                pivotA7 = btVector3(btScalar(0.0f), btScalar(-fPreLegLength), btScalar(0.0f));
                pivotB7 = btVector3(btScalar(0.0f), btScalar(fLegLength), btScalar(0.0f));
                btVector3 axisA7 = btVector3(btScalar(0.0f), btScalar(0.0f), btScalar(-1.0f));//setAxis('y');
                btVector3 axisB7 = btVector3(btScalar(1.0f), btScalar(0.0f), btScalar(0.0f));//setAxis('x');
                
                btHingeConstraint* hingeC7 = new btHingeConstraint(*m_bodies[1 + 3 * i], *m_bodies[2 + 3 * i], pivotA7, pivotB7, axisA7, axisB7);
                hingeC7->setLimit(-M_PI_8, -M_PI_8);//setLimit(-2*M_PI_8, 0);//inverse of limits
                //hingeC4->setLimit(-M_PI_8, M_PI_8);//M_PI_8
                m_joints[1 + 3 * i] = hingeC7;
                m_ownerWorld->addConstraint(m_joints[1 + 3 * i], true);
                //computeConstraints(pivotA, 'y', pivotB, 'y', 'y', btScalar(0), btScalar(0), 1 + 3 * i, 2 + 3 * i, 1 + 3 * i);
                /********************************************************************************************************/
                // knee joints2 //
                pivotA8 = btVector3(btScalar(0.0f), btScalar(-fLegLength / 2), btScalar(0.0f));
                pivotB8 = btVector3(btScalar(0.0f), btScalar(fForeLegLength / 2), btScalar(0.0f));
                btVector3 axisA8 = btVector3(btScalar(-1.0f), btScalar(0.0f), btScalar(0.0f));//setAxis('y');
                btVector3 axisB8 = btVector3(btScalar(0.0f), btScalar(0.0f), btScalar(1.0f));//setAxis('x');
                
                btHingeConstraint* hingeC8 = new btHingeConstraint(*m_bodies[2 + 3 * i], *m_bodies[3 + 3 * i], pivotA8, pivotB8, axisA8, axisB8);
                hingeC8->setLimit(-2 * M_PI_4, -2*M_PI_4);//setLimit(-M_PI_4, -M_PI_4);//-2 * M_PI_4
                //hingeC5->setLimit(-2 * M_PI_4, -M_PI_4);//2*M_PI_4
                m_joints[2 + 3 * i] = hingeC8;
                m_ownerWorld->addConstraint(m_joints[2 + 3 * i], true);
                //computeConstraints(pivotA, 'y', pivotB, 'y', 'y', btScalar(0), btScalar(0), 2 + 3 * i, 3 + 3 * i, 2 + 3 * i);
                /********************************************************************************************************/
            }
            
            if (i == 3) {
                // setup start position
                btVector3 origin13 =  btVector3(btScalar(0.), fHeight, fBodySize.getZ() + (fPreLegLength));
                transform.setIdentity();
                transform.setOrigin(origin13);
                
                btVector3 vToBone13 = (origin13 - vRoot).normalize(); ////for 1,2,4,5 vToBone.cross(vUp) = vAxis
                btVector3 vAxis13 = vToBone13.cross(vUp); //
                transform.setRotation(btQuaternion(vAxis13, M_PI_2)); //
                m_bodies[1 + 3 * i] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[1 + 3 * i]);
                //setStartPosition(origin1, true, 1 + 3 * i );
                /********************************************************************************************************/
                btVector3 origin23 = btVector3(btScalar(0.), fHeight, fBodySize.getZ() + fPreLegLength + fLegLength);
                transform.setIdentity();
                transform.setOrigin(origin23);
                
                btVector3 vToBone23 = (origin23 - vRoot).normalize(); ////for 1,2,4,5 vToBone.cross(vUp) = vAxis
                btVector3 vAxis23 = vToBone23.cross(vUp); //
                transform.setRotation(btQuaternion(vAxis23, M_PI_2)); //
                m_bodies[2 + 3 * i] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[2 + 3 * i]);
                //setStartPosition(origin2, true, 2 + 3 * i );
                /********************************************************************************************************/
                btVector3 origin33 =(btVector3(btScalar(0.), fHeight - (fForeLegLength / 2), fBodySize.getZ() + fPreLegLength + fLegLength + (fForeLegLength / 2)));
                transform.setIdentity();
                transform.setOrigin(origin33);
                
                btVector3 vToBone33 = (origin33 - vRoot).normalize(); ////for 1,2,4,5 vToBone.cross(vUp) = vAxis
                btVector3 vAxis33 = vToBone33.cross(vUp); //
                //transform.setRotation(btQuaternion(vAxis, M_PI_2)); //
                m_bodies[3 + 3 * i] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[3 + 3 * i]);
                //setStartPosition(origin3, false, 3 + 3 * i );
                /********************************************************************************************************/
                // setup constraints
                // hip joints //
                pivotA9 = btVector3(btScalar(0.0f), btScalar(0.0f), btScalar(fBodySize.getZ()));
                pivotB9 = btVector3(btScalar(0.0f), btScalar(-fPreLegLength), btScalar(0.0f));
                btVector3 axisA9 = btVector3(btScalar(0.0f), btScalar(1.0f), btScalar(0.0f));//change sign ?-1
                btVector3 axisB9 = btVector3(btScalar(1.0f), btScalar(0.0f), btScalar(0.0f));//setAxis('x');
                
                btHingeConstraint* hingeC9 = new btHingeConstraint(*m_bodies[0], *m_bodies[1 + 3 * i], pivotA9, pivotB9, axisA9, axisB9);
                hingeC9->setLimit(-M_PI_2, -M_PI_2);//setLimit(-2 * M_PI / 3, -M_PI_4);//setLimit(3*M_PI / 2, 3*M_PI / 2);
                //hingeC6->setLimit(3 * M_PI / 2, 3 * M_PI / 2);//4*PI/3
                m_joints[3 * i] = hingeC9;
                m_ownerWorld->addConstraint(m_joints[3 * i], true);
                //computeConstraints(pivotA, 'y', pivotB, 'y', 'y', btScalar(0), btScalar(0), 0, 1 + 3 * i, 3 * i);
                /********************************************************************************************************/
                // knee joints //
                pivotA10 = btVector3(btScalar(0.0f), btScalar(fPreLegLength), btScalar(0.0f));
                pivotB10 = btVector3(btScalar(0.0f), btScalar(-fLegLength), btScalar(0.0f));
                btVector3 axisA10 = btVector3(btScalar(0.0f), btScalar(0.0f), btScalar(-1.0f));//change the sighn
                btVector3 axisB10 = btVector3(btScalar(1.0f), btScalar(0.0f), btScalar(0.0f));//setAxis('y');
                //btVector3 axisA1 = btVector3(btScalar(0.0f), btScalar(0.0f), btScalar(-1.0f));//setAxis('y');
                //btVector3 axisB1 = btVector3(btScalar(1.0f), btScalar(0.0f), btScalar(0.0f));//setAxis('y');
                btHingeConstraint* hingeC10 = new btHingeConstraint(*m_bodies[1 + 3 * i], *m_bodies[2 + 3 * i], pivotA10, pivotB10, axisA10, axisB10);
                hingeC10->setLimit(-M_PI_8, -M_PI_8);//btScalar(0.0f), btScalar(0.0f));
                //hingeC7->setLimit(-M_PI_8, M_PI_8);
                m_joints[1 + 3 * i] = hingeC10;
                m_ownerWorld->addConstraint(m_joints[1 + 3 * i], true);
                //computeConstraints(pivotA, 'y', pivotB, 'y', 'y', btScalar(0), btScalar(0), 1 + 3 * i, 2 + 3 * i, 1 + 3 * i);
                /********************************************************************************************************/
                // knee joints2 //
                pivotA11 = btVector3(btScalar(0.0f), btScalar(fLegLength / 2), btScalar(0.0f));
                pivotB11 = btVector3(btScalar(0.0f), btScalar(fForeLegLength / 2), btScalar(0.0f));
                btVector3 axisA11 = btVector3(btScalar(-1.0f), btScalar(0.0f), btScalar(0.0f));//change sight 1
                btVector3 axisB11 = btVector3(btScalar(0.0f), btScalar(0.0f), btScalar(1.0f));//setAxis('y');
                btHingeConstraint* hingeC11 = new btHingeConstraint(*m_bodies[2 + 3 * i], *m_bodies[3 + 3 * i], pivotA11, pivotB11, axisA11, axisB11);
                hingeC11->setLimit(M_PI_2, M_PI_2);//2 * M_PI / 3);//setLimit(-2 *M_PI_4, -2*M_PI_4);//setLimit(M_PI_2, 2*M_PI/3);//M_PI_4, 2*M_PI_4);
                //hingeC8->setLimit(-2 * M_PI_4, -M_PI_4);
                m_joints[2 + 3 * i] = hingeC11;
                m_ownerWorld->addConstraint(m_joints[2 + 3 * i], true);
                //computeConstraints(pivotA, 'y', pivotB, 'y', 'y', btScalar(0), btScalar(0), 2 + 3 * i, 3 + 3 * i, 2 + 3 * i);
            }
            
            if (i == 4) {
                // setup start position
                btVector3 origin14 =  btVector3(-fBodySize.getX() - (fPreLegLength / 2)* cos(fAngle), fHeight, fBodySize.getZ() + sin(fAngle)*(fPreLegLength / 2));
                transform.setIdentity();
                transform.setOrigin(origin14);
                
                btVector3 vToBone14 = (origin14 - vRoot).normalize(); ////for 1,2,4,5 vToBone.cross(vUp) = vAxis
                btVector3 vAxis14 = vToBone14.cross(vUp); //
                transform.setRotation(btQuaternion(vAxis14, M_PI_2)); //
                m_bodies[1 + 3 * i] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[1 + 3 * i]);
                //setStartPosition(origin1, true, 1 + 3 * i );
                /********************************************************************************************************/
                btVector3 origin24 = btVector3( -fBodySize.getX() - fPreLegLength*cos(fAngle) - fLegLength*cos(fAngle),
                                               fHeight,
                                               fBodySize.getZ() + sin(fAngle)*fPreLegLength + sin(fAngle)*fLegLength);
                transform.setIdentity();
                transform.setOrigin(origin24);
                
                btVector3 vToBone24 = (origin24 - vRoot).normalize(); ////for 1,2,4,5 vToBone.cross(vUp) = vAxis
                btVector3 vAxis24 = vToBone24.cross(vUp); //
                transform.setRotation(btQuaternion(vAxis24, M_PI_2)); //
                m_bodies[2 + 3 * i] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[2 + 3 * i]);
                //setStartPosition(origin2, true, 2 + 3 * i );
                /********************************************************************************************************/
                btVector3 origin34 =(btVector3(-fBodySize.getX() - fPreLegLength*cos(fAngle) -fLegLength*cos(fAngle) -(fForeLegLength / 2)*cos(fAngle),
                                               fHeight - (fForeLegLength / 2),
                                               fBodySize.getZ() + fPreLegLength*sin(fAngle) + fLegLength*sin(fAngle) + (fForeLegLength / 2)*sin(fAngle)));
                transform.setIdentity();
                transform.setOrigin(origin34);
                
                btVector3 vToBone34 = (origin34 - vRoot).normalize(); ////for 1,2,4,5 vToBone.cross(vUp) = vAxis
                btVector3 vAxis34 = vToBone34.cross(vUp); //
                //transform.setRotation(btQuaternion(vAxis22, M_PI_2)); //
                m_bodies[3 + 3 * i] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[3 + 3 * i]);
                //setStartPosition(origin3, false, 3 + 3 * i );
                
                // setup constraints
                // hip joints //
                pivotA12 = btVector3(btScalar(-fBodySize.getX()), btScalar(0.0f), btScalar(fBodySize.getZ()));
                pivotB12 = btVector3(btScalar(0.0f), btScalar(-fPreLegLength), btScalar(0.0f));
                btVector3 axisA12 = btVector3(btScalar(0.0f), btScalar(1.0f), btScalar(0.0f));//-1
                btVector3 axisB12 = btVector3(btScalar(1.0f), btScalar(0.0f), btScalar(0.0f));//setAxis('x');
                
                btHingeConstraint* hingeC12 = new btHingeConstraint(*m_bodies[0], *m_bodies[1 + 3 * i], pivotA12, pivotB12, axisA12, axisB12);
                hingeC12->setLimit(-M_PI_4, -M_PI_4);//setLimit(-2 * M_PI / 3, -M_PI_4);//setLimit(-M_PI_4, -M_PI_4);//setLimit(4 * M_PI / 3, 3 * M_PI / 2);
                //hingeC6->setLimit(3 * M_PI / 2, 3 * M_PI / 2);//4*PI/3
                m_joints[3 * i] = hingeC12;
                m_ownerWorld->addConstraint(m_joints[3 * i], true);
                //computeConstraints(pivotA, 'y', pivotB, 'y', 'y', btScalar(0), btScalar(0), 0, 1 + 3 * i, 3 * i);
                /********************************************************************************************************/
                // knee joints //
                pivotA13 = btVector3(btScalar(0.0f), btScalar(fPreLegLength), btScalar(0.0f));
                pivotB13 = btVector3(btScalar(0.0f), btScalar(-fLegLength), btScalar(0.0f));
                btVector3 axisA13 = btVector3(btScalar(0.0f), btScalar(0.0f), btScalar(-1.0f));//1
                btVector3 axisB13 = btVector3(btScalar(1.0f), btScalar(0.0f), btScalar(0.0f));//setAxis('x');
                
                btHingeConstraint* hingeC13 = new btHingeConstraint(*m_bodies[1 + 3 * i], *m_bodies[2 + 3 * i], pivotA13, pivotB13, axisA13, axisB13);
                hingeC13->setLimit(-M_PI_8, -M_PI_8);
                //hingeC7->setLimit(-M_PI_8, M_PI_8);
                m_joints[1 + 3 * i] = hingeC13;
                m_ownerWorld->addConstraint(m_joints[1 + 3 * i], true);
                //computeConstraints(pivotA, 'y', pivotB, 'y', 'y', btScalar(0), btScalar(0), 1 + 3 * i, 2 + 3 * i, 1 + 3 * i);
                /********************************************************************************************************/
                // knee joints2 //
                pivotA14 = btVector3(btScalar(0.0f), btScalar(fLegLength / 2), btScalar(0.0f));
                pivotB14 = btVector3(btScalar(0.0f), btScalar(fForeLegLength / 2), btScalar(0.0f));
                btVector3 axisA14 = btVector3(btScalar(-1.0f), btScalar(0.0f), btScalar(0.0f));//change sight 1
                btVector3 axisB14 = btVector3(btScalar(0.0f), btScalar(0.0f), btScalar(1.0f));
                
                btHingeConstraint* hingeC14 = new btHingeConstraint(*m_bodies[2 + 3 * i], *m_bodies[3 + 3 * i], pivotA14, pivotB14, axisA14, axisB14);
                hingeC14->setLimit(M_PI_2, M_PI_2);//setLimit(M_PI_2, 2 * M_PI / 3);//setLimit(-2 * M_PI_4, -M_PI_4);
                //hingeC11->setLimit(M_PI_2, 2 * M_PI / 3);
                m_joints[2 + 3 * i] = hingeC14;
                m_ownerWorld->addConstraint(m_joints[2 + 3 * i], true);
                //computeConstraints(pivotA, 'y', pivotB, 'y', 'y', btScalar(0), btScalar(0), 2 + 3 * i, 3 + 3 * i, 2 + 3 * i);
            }
            
            if (i == 5) {
                // setup start position
                btVector3 origin15 =  btVector3( fBodySize.getX() + (fPreLegLength)* cos(fAngle), fHeight, fBodySize.getZ() + sin(fAngle)*(fPreLegLength));
                transform.setIdentity();
                transform.setOrigin(origin15);
                
                btVector3 vToBone15 = (origin15 - vRoot).normalize(); ////for 1,2,4,5 vToBone.cross(vUp) = vAxis
                btVector3 vAxis15 = vToBone15.cross(vUp); //
                transform.setRotation(btQuaternion(vAxis15, M_PI_2)); //
                m_bodies[1 + 3 * i] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[1 + 3 * i]);
                //setStartPosition(origin1, true, 1 + 3 * i );
                /********************************************************************************************************/
                btVector3 origin25 = btVector3(btVector3(
                                                         fBodySize.getX() + fPreLegLength*cos(fAngle) + fLegLength*cos(fAngle),
                                                         fHeight,
                                                         fBodySize.getZ() + sin(fAngle)*fPreLegLength + sin(fAngle)*fLegLength));
                transform.setIdentity();
                transform.setOrigin(origin25);
                
                btVector3 vToBone25 = (origin25 - vRoot).normalize(); ////for 1,2,4,5 vToBone.cross(vUp) = vAxis
                btVector3 vAxis25 = vToBone25.cross(vUp); //
                transform.setRotation(btQuaternion(vAxis25, M_PI_2)); //
                m_bodies[2 + 3 * i] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[2 + 3 * i]);
                //setStartPosition(origin2, true, 2 + 3 * i );
                /********************************************************************************************************/
                btVector3 origin35 =(btVector3(fBodySize.getX() + fPreLegLength*cos(fAngle) + fLegLength*cos(fAngle) + (fForeLegLength / 2)*cos(fAngle),
                                               fHeight - (fForeLegLength / 2),
                                               fBodySize.getZ() + fPreLegLength*sin(fAngle) + fLegLength*sin(fAngle) + (fForeLegLength / 2)*sin(fAngle)));
                transform.setIdentity();
                transform.setOrigin(origin35);
                
                btVector3 vToBone35 = (origin35 - vRoot).normalize(); ////for 1,2,4,5 vToBone.cross(vUp) = vAxis
                btVector3 vAxis35 = vToBone35.cross(vUp); //
                //transform.setRotation(btQuaternion(vAxis32, M_PI_2)); //
                m_bodies[3 + 3 * i] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[3 + 3 * i]);
                //setStartPosition(origin3, false, 3 + 3 * i );
                /********************************************************************************************************/
                // setup constraints
                // hip joints //
                pivotA15 = btVector3(btScalar(fBodySize.getX()), btScalar(0.0f), btScalar(fBodySize.getZ()));
                pivotB15 = btVector3(btScalar(0.0f), btScalar(-fPreLegLength), btScalar(0.0f));
                btVector3 axisA15 = btVector3(btScalar(0.0f), btScalar(1.0f), btScalar(0.0f));//change sign
                btVector3 axisB15 = btVector3(btScalar(1.0f), btScalar(0.0f), btScalar(0.0f));//setAxis('x');
                
                btHingeConstraint* hingeC15 = new btHingeConstraint(*m_bodies[0], *m_bodies[1 + 3 * i], pivotA15, pivotB15, axisA15, axisB15);
                hingeC15->setLimit(-M_PI_2, -M_PI_2);//setLimit(-2 * M_PI / 3, -M_PI_4);//setLimit(3 * M_PI / 2, 3 * M_PI / 2);//setLimit(4 * M_PI / 3, 3 * M_PI / 2);
                //hingeC12->setLimit(3 * M_PI / 2, 3 * M_PI / 2);//4*PI/3
                m_joints[3 * i] = hingeC15;
                m_ownerWorld->addConstraint(m_joints[3 * i], true);
                //computeConstraints(pivotA, 'y', pivotB, 'y', 'y', btScalar(0), btScalar(0), 0, 1 + 3 * i, 3 * i);
                /********************************************************************************************************/
                // knee joints //
                pivotA16 = btVector3(btScalar(0.0f), btScalar(fPreLegLength), btScalar(0.0f));
                pivotB16 = btVector3(btScalar(0.0f), btScalar(-fLegLength), btScalar(0.0f));
                btVector3 axisA16 = btVector3(btScalar(0.0f), btScalar(0.0f), btScalar(-1.0f));//change sign
                btVector3 axisB16 = btVector3(btScalar(1.0f), btScalar(0.0f), btScalar(0.0f));//setAxis('x');
                
                btHingeConstraint* hingeC16 = new btHingeConstraint(*m_bodies[1 + 3 * i], *m_bodies[2 + 3 * i], pivotA16, pivotB16, axisA16, axisB16);
                hingeC16->setLimit(-M_PI_8, -M_PI_8);
                //hingeC13->setLimit(-M_PI_8, M_PI_8);
                m_joints[1 + 3 * i] = hingeC16;
                m_ownerWorld->addConstraint(m_joints[1 + 3 * i], true);
                //computeConstraints(pivotA, 'y', pivotB, 'y', 'y', btScalar(0), btScalar(0), 1 + 3 * i, 2 + 3 * i, 1 + 3 * i);
                /********************************************************************************************************/
                // knee joints2 //
                pivotA17 = btVector3(btScalar(0.0f), btScalar(fLegLength / 2), btScalar(0.0f));
                pivotB17 = btVector3(btScalar(0.0f), btScalar(fForeLegLength / 2), btScalar(0.0f));
                btVector3 axisA17 = btVector3(btScalar(-1.0f), btScalar(0.0f), btScalar(0.0f));//change sign
                btVector3 axisB17 = btVector3(btScalar(0.0f), btScalar(0.0f), btScalar(1.0f));//setAxis('x');
                
                btHingeConstraint* hingeC17 = new btHingeConstraint(*m_bodies[2 + 3 * i], *m_bodies[3 + 3 * i], pivotA17, pivotB17, axisA17, axisB17);
                hingeC17->setLimit(M_PI_2, M_PI_2);//setLimit(M_PI_2, 2 * M_PI / 3);//setLimit(M_PI_4, 2 * M_PI_4);//(-2 * M_PI_4, -M_PI_4);
                //hingeC14->setLimit(M_PI_2, 2 * M_PI / 3);
                m_joints[2 + 3 * i] = hingeC17;
                m_ownerWorld->addConstraint(m_joints[2 + 3 * i], true);
                //computeConstraints(pivotA, 'y', pivotB, 'y', 'y', btScalar(0), btScalar(0), 2 + 3 * i, 3 + 3 * i, 2 + 3 * i);
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
    
    
    virtual    ~TestRig()
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
    
    //    m_fMuscleStrength = 0.05f;
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
    
    int joints_1[] = {7,10,4};
    int joints_2[] = {6,9,3};
    
    int joints_3[] = {16,1,13};
    int joints_4[] = {15,0,12};
    
    int joints_5[] = {8,11,5};
    int joints_6[] = {17,2,14};
    
    Limits limits_0( -2 * M_PI/3, -M_PI_4 );
    Limits limits_1( -M_PI_8, M_PI_8 );
    
    Limits limits_array_0[3] = {limits_0, limits_0, limits_0}; // 6 9 3   15 0 12
    Limits limits_array_1[3] = {limits_1, limits_1, limits_1}; // 7 10 4  16 1 13
    
    
    Stage* stage0 = new Stage(joints_1, limits_array_1, Directions::forward, joints_3, limits_array_1, Directions::backward);
    stages.push_back(*stage0);
    Stage* stage1 = new Stage(joints_2, limits_array_0, Directions::forward, joints_4, limits_array_0, Directions::backward);
    stages.push_back(*stage1);
    Stage* stage2 = new Stage(joints_1, limits_array_1, Directions::backward, joints_3, limits_array_1, Directions::forward);
    stages.push_back(*stage2);
    Stage* stage3 = new Stage(joints_2, limits_array_0, Directions::backward, joints_4, limits_array_0, Directions::forward);
    stages.push_back(*stage3);
    
    current_stage = 0;
}


void MotorDemo::spawnTestRig(const btVector3& startOffset, bool bFixed)
{
    TestRig* rig = new TestRig(m_dynamicsWorld, startOffset, bFixed);
    m_rigs.push_back(rig);
}

void    PreStep()
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
    
    
    float current_angle1;
    float current_angle2;
    
    Stage cur_stage = stages[current_stage];
    
    
    if (isfinished_1 && isfinished_2)
    {
        current_stage++;
        isfinished_1 = false;
        isfinished_2 = false;
    }
    
    if (current_stage >= 4) current_stage = 0;
    
    std::cout<<"Current stage: "<<current_stage<<std::endl;
    
    
    time_passed += ms/1000000.;
    
    if (time_passed >= 5)
    {
        for (int r = 0; r<m_rigs.size(); r++)
        {
            for (int i = 0; i <3 ; i++)
            {
                int current_joint1 = cur_stage.getHinges_1()[i];
                btScalar current_up_limit1 = cur_stage.getLimits_1()[i].upLimit;
                btScalar current_low_limit1 = cur_stage.getLimits_1()[i].lowLimit;
                Directions direction1 = cur_stage.getDirection_1();
                
                
                btHingeConstraint* hingeC = static_cast<btHingeConstraint*>(m_rigs[r]->GetJoints()[current_joint1]);
                btScalar fCurAngle = hingeC->getHingeAngle();
                btScalar fTargetPercent = (int(m_Time / 1000) % int(m_fCyclePeriod)) / m_fCyclePeriod;
                btScalar fTargetAngle = 0.5 * (1 + sin(2 * M_PI * fTargetPercent)); // change
                btScalar fTargetLimitAngle = hingeC->getLowerLimit() + fTargetAngle * (hingeC->getUpperLimit() - hingeC->getLowerLimit());
                btScalar fAngleError = fTargetLimitAngle - fCurAngle;
                btScalar fDesiredAngularVel = 1000000.f * fAngleError / ms; //change
                hingeC->setLimit(current_low_limit1, current_up_limit1);
                hingeC->enableAngularMotor(true, fDesiredAngularVel*10, m_fMuscleStrength);
                current_angle1 = hingeC->getHingeAngle();
                
                std::cout<<"i = "<<i<<" Current angle1 = "<<current_angle1<<std::endl;
                
                
                if ( direction1==Directions::forward )
                {
                    if (current_angle1 >= current_up_limit1 - abs (current_up_limit1/10))
                    {
                        isfinished_1 = true;
                        hingeC->setLimit(current_up_limit1, current_up_limit1);
                    }
                    
                }
                
                else
                {
                    if (current_angle1 <= current_low_limit1 + abs (current_low_limit1/10))
                    {
                        isfinished_1 = true;
                        hingeC->setLimit(current_low_limit1, current_low_limit1);
                    }
                }
                /////////////////////////////////////////
                int current_joint2 = cur_stage.getHinges_2()[i];
                btScalar current_up_limit2 = cur_stage.getLimits_2()[i].upLimit;
                btScalar current_low_limit2 = cur_stage.getLimits_2()[i].upLimit;
                Directions direction2 = cur_stage.getDirection_2();
                
                btHingeConstraint* hingeC1 = static_cast<btHingeConstraint*>(m_rigs[r]->GetJoints()[current_joint2]);
                btScalar fCurAngle1 = hingeC1->getHingeAngle();
                btScalar fTargetPercent1 = (int(m_Time / 1000) % int(m_fCyclePeriod)) / m_fCyclePeriod;
                btScalar fTargetAngle1 = 0.5 * (1 + sin(2 * M_PI * fTargetPercent1)); // change
                btScalar fTargetLimitAngle1 = hingeC1->getLowerLimit() + fTargetAngle1 * (hingeC1->getUpperLimit() - hingeC1->getLowerLimit());
                btScalar fAngleError1 = fTargetLimitAngle1 - fCurAngle1;
                btScalar fDesiredAngularVel1 = 1000000.f * fAngleError1 / ms; //change
                hingeC1->setLimit(current_low_limit2, current_up_limit2);
                hingeC1->enableAngularMotor(true, fDesiredAngularVel1*10, m_fMuscleStrength);
                current_angle2 = hingeC1->getHingeAngle();
                
                std::cout<<"i = "<<i<<" Current angle2 = "<<current_angle2<<std::endl;
                
                if ( direction2 == Directions::forward )
                {
                    if (current_angle2 >= current_up_limit2 - abs (current_up_limit2/10))
                    {
                        isfinished_2 = true;
                        hingeC1->setLimit(current_up_limit2, current_up_limit2);
                    }
                }
                
                else
                {
                    if (current_angle2 <= current_low_limit2 + abs (current_low_limit2/10))
                    {
                        isfinished_2 = true;
                        hingeC1->setLimit(current_low_limit2, current_low_limit2);
                    }
                }
                //
                
                
                
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

