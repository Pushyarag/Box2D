/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

/*
 * Base code for CS 251 Software Systems Lab
 * Department of Computer Science and Engineering, IIT Bombay
 *
 */


#include "cs251_base.hpp"
#include "render.hpp"


#include <cmath>
#define PI 3.14159265


#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include "GL/freeglut.h"
#endif

#include <cstring>
using namespace std;

#include "dominos.hpp"

namespace cs251
{
/*!  ground object
       * \brief the class to create ground
       */
class ground
{


public:

    ground(b2World* m_world)
    {
        b2Body* b1;
        b2EdgeShape shape;
        shape.Set(b2Vec2(-90.0f, 0.0f), b2Vec2(180.0f, 0.0f));
        b2BodyDef bd;
        b1 = m_world->CreateBody(&bd);
        b1->CreateFixture(&shape, 0.0f);
    }

};





/*!  weld joint object
       * \brief the class to create a fixed joint between 2 given objects
       */

class weld{
public:
  weld(b2Body* b1,b2Body* b2,float x,float y,b2World* m_world){
b2WeldJointDef weldJointDef;
weldJointDef.collideConnected = true;
b2Vec2 anchor;
      anchor.Set(x, y);
weldJointDef.Initialize(b1, b2, anchor);
m_world->CreateJoint(&weldJointDef);
}
};

/*!  revolute joint object
       * \brief the class to create a joint between 2 given objects which allows free rotation about the joint
       */
class rev_j{
public:
  rev_j(b2Body* b1,b2Body* b2,float x,float y,b2World* m_world,int mot=0){


b2RevoluteJointDef jd;


if(mot!=0){
jd.enableMotor = true;
jd.motorSpeed = mot;
jd.maxMotorTorque = 1000;
}
      b2Vec2 anchor;
      anchor.Set(x, y);
      jd.Initialize(b1, b2, anchor);
      m_world->CreateJoint(&jd);
 }
};



/*!  sphere object
       * \brief This is the class to create spheres
       */
class sphere
{

public:

    sphere(float radius,float density,float friction, float restitution, float x, float y,bool gravity, b2World* m_world)
    {

 b2Body* sbody;
        b2CircleShape circle;
        circle.m_radius = radius;

        b2FixtureDef ballfd;
        ballfd.shape = &circle;
        ballfd.density = density;
        ballfd.friction = friction;
        ballfd.restitution = restitution;
        b2BodyDef ballbd;
        ballbd.type = b2_dynamicBody;
        ballbd.position.Set(x,y);
        sbody = m_world->CreateBody(&ballbd);
        sbody->CreateFixture(&ballfd);
        if(gravity==0)
        {
            sbody->SetGravityScale(-0.3);
        }
        else{
            if(gravity==2){
            sbody->SetGravityScale(2.3);
            }
        }
    }
};

/*!  block object
       * \brief This is the class to create blocks
       */

class block
{


public:
    b2Body* body;
    b2BodyDef bd;

    block(float w,float h,float x,float y,int a, b2World* m_world,int s_d=0, float friction =1.0f,float density=1.0f)
    {

        b2PolygonShape shape;
        shape.SetAsBox(w, h);


        bd.angle = a; //set the starting angle
        bd.position.Set(x, y);
        if(s_d==1)bd.type = b2_dynamicBody;
        if(s_d==0)bd.type = b2_staticBody;
        body = m_world->CreateBody(&bd);
        b2FixtureDef *fd = new b2FixtureDef;
        fd->density = density;
        fd->shape = new b2PolygonShape;
        fd->shape = &shape;
        fd->friction = friction;
        fd->restitution = 0.0f;
        body->CreateFixture(fd);

    }
};





class CircleQuarter{
public:
CircleQuarter(float x, float y, float radius, b2World * m_world, int start, int ends){

for (int i=start ; i <= ends; i++) 
{

new block( 0.100f,0.00f,x+radius*cos( i * PI / 180.0),y+radius*sin( i * PI / 180.0),-i, m_world,0,0.0f,1.0f);
}

}
};






/*!  pulley object
       * \brief This is the class to create pulleys
       */

class pulley
{


public:

    pulley(float x,float y, float l_density, float r_density, b2World *m_world)
    {
        b2Vec2 shift = b2Vec2(x,y);
        b2BodyDef *bd = new b2BodyDef;
        bd->type = b2_dynamicBody;
        bd->position.Set(-10+x,15+y);
        bd->fixedRotation = true;

        //The open box
        b2FixtureDef *fd1 = new b2FixtureDef;
        fd1->density = l_density;///default 10.0f
        fd1->friction = 0.5;
        fd1->restitution = 0.f;
        fd1->shape = new b2PolygonShape;
        b2PolygonShape bs1;
        bs1.SetAsBox(2,0.2, b2Vec2(0.f,-1.9f)+shift, 0);
        fd1->shape = &bs1;
        b2FixtureDef *fd2 = new b2FixtureDef;
        fd2->density = l_density;///default 10.0f
        fd2->friction = 0.5;
        fd2->restitution = 0.f;
        fd2->shape = new b2PolygonShape;
        b2PolygonShape bs2;
        bs2.SetAsBox(0.2,2, b2Vec2(2.0f,0.f)+shift, 0);
        fd2->shape = &bs2;
        b2FixtureDef *fd3 = new b2FixtureDef;
        fd3->density = l_density;///default 10.0f
        fd3->friction = 0.5;
        fd3->restitution = 0.f;
        fd3->shape = new b2PolygonShape;
        b2PolygonShape bs3;
        bs3.SetAsBox(0.2,2, b2Vec2(-2.0f,0.f)+shift, 0);
        fd3->shape = &bs3;

        b2Body* box1 = m_world->CreateBody(bd);
        box1->CreateFixture(fd1);
        box1->CreateFixture(fd2);
        box1->CreateFixture(fd3);

        //The bar
        bd->position.Set(25+x,15+y);
        fd1->density = r_density;///default 34.0f
        b2Body* box2 = m_world->CreateBody(bd);
        box2->CreateFixture(fd1);

        // The pulley joint
        b2PulleyJointDef* myjoint = new b2PulleyJointDef();
        b2Vec2 worldAnchorOnBody1(-10, 15); // Anchor point on body 1 in world axis
        b2Vec2 worldAnchorOnBody2(25, 15); // Anchor point on body 2 in world axis
        b2Vec2 worldAnchorGround1(-10+x, 20+y); // Anchor point for ground 1 in world axis
        b2Vec2 worldAnchorGround2(25+x, 20+y); // Anchor point for ground 2 in world axis
        float32 ratio = 1.0f; // Define ratio
        myjoint->Initialize(box1, box2, worldAnchorGround1+shift, worldAnchorGround2+shift, box1->GetWorldCenter(), box2->GetWorldCenter(), ratio);
        m_world->CreateJoint(myjoint);
    }
};






/*!  see_saw object
       * \brief This is the class to create see_saw
       */

class see_saw
{


public:

    see_saw(b2World* m_world)
    {

        //The triangle wedge
        b2Body* sbody;
        b2PolygonShape poly;
        b2Vec2 vertices[3];
        vertices[0].Set(-1,0);
        vertices[1].Set(1,0);
        vertices[2].Set(0,4.5);
        poly.Set(vertices, 3);
        b2FixtureDef wedgefd;
        wedgefd.shape = &poly;
        wedgefd.density = 10.0f;
        wedgefd.friction = 0.0f;
        wedgefd.restitution = 0.0f;
        b2BodyDef wedgebd;
        wedgebd.position.Set(-30.0f, 0.0f);
        sbody = m_world->CreateBody(&wedgebd);
        sbody->CreateFixture(&wedgefd);

        //The plank on top of the wedge
        b2PolygonShape shape;
        shape.SetAsBox(8.0f, 0.2f);
        b2BodyDef bd2;
        bd2.position.Set(-30.0f, 4.5f);
        bd2.type = b2_dynamicBody;
        b2Body* body = m_world->CreateBody(&bd2);
        b2FixtureDef *fd2 = new b2FixtureDef;
        fd2->density = 0.01f;
        fd2->friction = 111.f;
        fd2->shape = new b2PolygonShape;
        fd2->shape = &shape;
        body->CreateFixture(fd2);

        b2RevoluteJointDef jd;
        b2Vec2 anchor;
        anchor.Set(-30.0f, 4.5f);
        jd.Initialize(sbody, body, anchor);
        m_world->CreateJoint(&jd);
    }
};


/*!  revolving_platform
       * \brief the class to create a block object hinged at its center
       */
class revolving_platform
{
public:

    revolving_platform(float h, float w, float x, float y,float density, b2World *m_world)
    {

        b2PolygonShape shape;
        shape.SetAsBox(2.2f+h, 0.2f+w);

        b2BodyDef bd;
        bd.position.Set(14.0f+x, 14.0f+y);
        bd.type = b2_dynamicBody;
        b2Body* body = m_world->CreateBody(&bd);
        b2FixtureDef *fd = new b2FixtureDef;
        fd->density = density;
        fd->shape = new b2PolygonShape;
        fd->shape = &shape;
        body->CreateFixture(fd);

        b2PolygonShape shape2;
        shape2.SetAsBox(0.2f+h, 2.0f+w);
        b2BodyDef bd2;
        bd2.position.Set(14.0f+x, 16.0f+y);
        b2Body* body2 = m_world->CreateBody(&bd2);

        b2RevoluteJointDef jointDef;
        jointDef.bodyA = body;
        jointDef.bodyB = body2;
        jointDef.localAnchorA.Set(0,0);
        jointDef.localAnchorB.Set(0,0);
        jointDef.collideConnected = false;
        m_world->CreateJoint(&jointDef);
    }
};


class custom_revolving_platform
{
public:

    custom_revolving_platform(float h, float w, float x, float y,float density, b2World *m_world)
    {

        b2PolygonShape shape;
        shape.SetAsBox(2.2f+h, 0.2f+w);

        b2BodyDef bd;
        bd.position.Set(14.0f+x, 14.0f+y);
        bd.type = b2_dynamicBody;
        b2Body* body = m_world->CreateBody(&bd);
        b2FixtureDef *fd = new b2FixtureDef;
        fd->density = density;
        fd->shape = new b2PolygonShape;
        fd->shape = &shape;
        body->CreateFixture(fd);

        b2PolygonShape shape2;
        shape2.SetAsBox(0.2f+h, 2.0f+w);
        b2BodyDef bd2;
        bd2.position.Set(14.0f+x, 16.0f+y);
        b2Body* body2 = m_world->CreateBody(&bd2);

        b2RevoluteJointDef jointDef;
        jointDef.bodyA = body;
        jointDef.bodyB = body2;
        jointDef.localAnchorA.Set((2.2f+h)/2,0+w/2);
        jointDef.localAnchorB.Set(0,0);
        jointDef.collideConnected = false;
        m_world->CreateJoint(&jointDef);
    }
};


/*!  dominoes
       * \brief the class to create a set of small rectangular blocks which when triggered fall one after the other
       */
class dominoes
{

public:

    dominoes(float h, float w, float x, float y,float gap, float density, float friction, int n, b2World *m_world)
    {
        b2PolygonShape shape;
        shape.SetAsBox(w, h);

        b2FixtureDef fd;
        fd.shape = &shape;
        fd.density = density;
        fd.friction = friction;
        fd.restitution = 0.001f;
        for (int i = 0; i < n; ++i)
        {
            b2BodyDef bd;
            bd.type = b2_dynamicBody;
            bd.position.Set(x + gap * i-0.45f, y);
            b2Body* body = m_world->CreateBody(&bd);
            body->CreateFixture(&fd);
        }
    }
};


/*!  pulley_joint object
       * \brief the class to create a pulley with 2 given bodies
       */
class pulley_j{
// The pulley joint
     public:
pulley_j(b2Body* box1,b2Body* box2,float b1x,float b1y,float b2x,float b2y,float g1x,float g1y,float g2x,float g2y,b2World* m_world){

      b2PulleyJointDef* myjoint = new b2PulleyJointDef();
      b2Vec2 worldAnchorOnBody1(b1x, b1y); // Anchor point on body 1 in world axis
      b2Vec2 worldAnchorOnBody2(b2x, b2y); // Anchor point on body 2 in world axis
      b2Vec2 worldAnchorGround1(g1x, g1y); // Anchor point for ground 1 in world axis
      b2Vec2 worldAnchorGround2(g2x, g2y); // Anchor point for ground 2 in world axis
      float32 ratio = 1.0f; // Define ratio
      myjoint->Initialize(box1, box2, worldAnchorGround1, worldAnchorGround2, box1->GetWorldCenter(), box2->GetWorldCenter(), ratio);
      m_world->CreateJoint(myjoint);

}

};


/*!  open box object
       * \brief the class to create a  box wit hno lid ie.. top side open
       */
class open_box{
public:
  b2Body* box1;
open_box(float x,float y, float l_density, float r_density, b2World *m_world)
    {
block* b1=new block(2.0f,0.2f,0.0f+x,-1.9f+y,0,m_world,1,1.0f,0.5f);
block* b2=new block(0.2f,2.0f,2.0f+x,y,0, m_world,1,1.0f,0.5f);
block* b3=new block(0.2f,2.0f,-2.0f+x,y,0,m_world,1,1.0f,0.5f);
b1->body->SetFixedRotation(true);
weld* w=new weld(b1->body,b2->body,x+2.0f, y-1.9f,m_world);
w=new weld(b3->body,b1->body,x-2.0f, y-1.9f,m_world);
box1=b1->body;
    }
};


class newtons_pendulum{
public:
newtons_pendulum(float x, float y, float radius,float density,float friction, float restitution, int n, float l, b2World * m_world){

//The pendulum that knocks the dominos off
for(int i=0;i<n;i++){
      b2Body* b2;
      {
	b2PolygonShape shape;
	shape.SetAsBox(0.25f, 1.5f);

	b2BodyDef bd;
	bd.position.Set(x, y-5.0f);
	b2 = m_world->CreateBody(&bd);
	b2->CreateFixture(&shape, 10.0f);
      }
/*
      b2Body* b4;
      {
	b2PolygonShape shape;
	shape.SetAsBox(0.5f, 0.25f);

	b2BodyDef bd;
	bd.type = b2_dynamicBody;
	bd.position.Set(-40.0f, 33.0f);
	b4 = m_world->CreateBody(&bd);
	b4->CreateFixture(&shape, 2.0f);
      }
*/

 b2Body* sbody;
        b2CircleShape circle;
        circle.m_radius = radius;

        b2FixtureDef ballfd;
        ballfd.shape = &circle;
        ballfd.density = density;
        ballfd.friction = friction;
        ballfd.restitution = restitution;
        b2BodyDef ballbd;
        ballbd.type = b2_dynamicBody;
        ballbd.position.Set(x,y);
        sbody = m_world->CreateBody(&ballbd);
        sbody->CreateFixture(&ballfd);
    //new sphere(1.0f,0.4f,0.5f,0.0f,19.5f+dominoes_and_block_x,10.1f+dominoes_and_block_y,1,m_world);
    //sbody-> SetGravityScale(-1);
      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      anchor.Set(x, y+5.0f);
      jd.Initialize(b2, sbody, anchor);
      m_world->CreateJoint(&jd);

      x=x+2*radius+0.01f;

}



}



};


/*!  Bucket_Water
       * \brief the class to create a bucket object filled with water
              */
class Bucket_Water{
public:
Bucket_Water(float x1,float y1,b2World* m_world){
  //Bucket of water
float x=x1;
float y=y1;
block* b;
weld* w;
rev_j* r;
b=new block(3.0f,0.2f,x,y+2.2f,30,m_world,1);
b2Body* b1=b->body;
// base of the bucket b2
b=new block(1.5f,0.2f,x+2.0f,y,0,m_world,1);
b2Body* b2=b->body;
w=new weld(b1,b2,x+0.5f, y,m_world);
b=new block(3.0f,0.2f,x+4.0f,y+2.2f,-30,m_world,1);
b2Body* b4=b->body;
w=new weld(b4,b2,x+3.5f, y,m_world);
//lid of bucket b5
b=new block(2.0f,0.2f,x+3.2f,y+4.8f,0,m_world,1);
b2Body* b5=b->body;
w=new weld(b5,b4,x+4.6f, y+4.8f,m_world);

//water
for(int i=0;i<20;i++){
for(int j=0;j<15;j++){
b=new block(0.1f,0.1f,x+0.5f+0.2f*j,y+1.2f+0.2f*i,30,m_world,1);
}
}

//pivoting the bucket
b=new block(5.0f,0.2f,x+7.5f,y,0,m_world,0);
b2Body* b3=b->body;
r=new rev_j(b2,b3,x+2.5f, y,m_world);

}

};


















block* b;
weld* w;
rev_j* r;
sphere* s;


/**  The is the constructor
* This is the documentation block for the constructor.
*/

dominos_t::dominos_t()
{
    //Ground
    /*! \var b1
     * \brief pointer to the body ground
     */

    new ground(m_world);






    new sphere(1.0f,5.f,0.0f,0.1f,-52.0f,22.0f,1,m_world);


    new block(10.0f,0.1f,-48.0f,20.0f,-60,m_world,0,1.0f,1.f);
    new block(1.0f,1.0f,-35.0f,4.5f,0,m_world,1,1.0f,0.05f);


    //The see-saw system at the bottom


    new see_saw(m_world);

    //The pulley system


    new pulley(-5.0f,5.0f,1.00f,3.0f,m_world);

    //The revolving horizontal platform

    new revolving_platform(0.8f,0.0f,5.5f,10.0f,1.0f,m_world);
    new sphere(1.0f,100.0f,0.0f,0.1f,19.5f,27.2f,2,m_world);
    //new sphere(1.0f,5.f,0.0f,0.1f,4.0f,25.2f,0,m_world);

    //Dominos


    float dominoes_and_block_x,dominoes_and_block_y;
    dominoes_and_block_x=-25.0f;
    dominoes_and_block_y=5.0f;

    new dominoes(1.0f,0.1f,10.0f+dominoes_and_block_x,10.0f+dominoes_and_block_y,1.0f,100.0f,0.1f,9,m_world);
    new block(6.0f,0.1f,14.5f+dominoes_and_block_x,10.0f+dominoes_and_block_y,0,m_world,0,1.0f,1.f);
    //new block(1.0f,1.0f,19.5f+dominoes_and_block_x,10.1f+dominoes_and_block_y,1.f,0,m_world,1);
new sphere(1.0f,10.0f,0.0f,0.0f,19.5f+dominoes_and_block_x,10.1f+dominoes_and_block_y,1,m_world);

    new revolving_platform(-2.0f,2.0f,-30.6f,-13.0f,100.0f,m_world);
    new revolving_platform(-2.0f,2.0f,-30.9f,-9.0f,100.0f,m_world);
    new revolving_platform(-2.0f,2.0f,-30.6f,-5.0f,100.0f,m_world);
    new revolving_platform(-2.0f,2.0f,-30.9f,-1.0f,100.0f,m_world);

    //new block(3.0f,0.2f,-1.7f,10.5f,-45,m_world,0, 1.0f,0.05f);
    new CircleQuarter(-5.0f,10.0f,5,m_world,0,90);
        new CircleQuarter(-5.0f,10.0f,7.4,m_world,0,90);
        new CircleQuarter(5.0f,10.0f,5,m_world,180,360);
        new CircleQuarter(5.0f,10.0f,2.4,m_world,180,360);
        new CircleQuarter(10.0f,10.0f,2.6,m_world,90,180);
new block(1.6,0.0,11.7f,10.0f,0,m_world,0,0.0f,1.0f);

        new CircleQuarter(33.0f,19.50f,10,m_world,90,130);
                new CircleQuarter(33.0f,19.50f,7.5,m_world,90,130);

        new CircleQuarter(33.0f,27.0f,2.5,m_world,0,90);
        new block(0.0f,12.0,33.0,15.0f,0,m_world,0,0.0f,1.0f);
                new block(0.0f,12.0,35.5,15.0f,0,m_world,0,0.0f,1.0f);
        new CircleQuarter(35.50f,2.5f,2.5,m_world,180,270);
new sphere(1.0,10.0,0.0,1.0,38.0,1.0f,1,m_world);
new sphere(1.0,10.0,0.0,1.0,42.0,1.0f,1,m_world);


//new sphere(1.0f,1.0f,0.0f,1.0f,33.0f,28.5f,0,m_world);
                //new CircleQuarter(49.5f,20.0f,10,m_world,180,270);
               // new CircleQuarter(30.5f,5.0f,5,m_world,90,270);





       // new block(15.0f,0.2f,3.7f,6.5f,0,m_world,0, 1.0f,0.05f);
        // new sphere(1.0f,1.0f,0.0f,1.0f,5.0f,7.7f,1,m_world);

new newtons_pendulum(46.0f,1.0f,1.0f,10.0f,0.0f,1.0f,10,0.0f,m_world);

    //  new sphere(1.0f,-1.7f,0.1f,4.0f,25.2f,0,m_world);
    new block(0.2f,3.0f,72.0f,3.0f,0,m_world,0,0.0f,1.0f);
new custom_revolving_platform(4,0,64.0f,-10.0f,10.0f,m_world);

{

      //The revolving Launcher

      float x=20.1f;
      float y=10.0f;
      float l=2.0f;
b=new block(l,0.2f,x,y,0,m_world,1);
b2Body* b1=b->body;
b=new block(0.01f,0.01f,x,y,0,m_world,0);
b2Body* b2=b->body;
r=new rev_j(b2,b1,x, y,m_world,0);

   b=new block(2.0f,0.2f,x+l+3.2f,y-1.6f,45,m_world,1);
  b2Body* b3=b->body;
  b=new block(2.0f,0.2f,x+l+1.2f,y-1.6f,-45,m_world,1);
b2Body* b4=b->body;
w=new weld(b1,b4,x+l, y,m_world);
w=new weld(b3,b4,x+l+2.0f, y-3.0f,m_world);


 b=new block(2.0f,0.2f,x-l-3.2f,y-1.6f,-45,m_world,1);
  b2Body* b5=b->body;
  b=new block(2.0f,0.2f,x-l-1.2f,y-1.6f,45,m_world,1);
b2Body* b6=b->body;
w=new weld(b1,b6,x-l, y,m_world);
w=new weld(b5,b6,x-l-2.0f, y-3.0f,m_world);
    }

float ballonx = 80.0f;
float ballony  = 3.0f;
new sphere(1.0f,5.f,0.0f,0.1f,ballonx,ballony,0,m_world);
 float rp_shiftx =  60.0f;

    new revolving_platform(3.0f,0.0f,4.0f+rp_shiftx,20.0f,10.0f,m_world);
    new sphere(1.0f,15.f,100.0f,0.01f,18.0f+rp_shiftx,37.2f,1,m_world);

    new revolving_platform(3.0f,0.0f,13.0f+rp_shiftx,19.5f,10.0f,m_world);
    new sphere(1.0f,15.f,100.0f,0.01f,27.0f+rp_shiftx,36.7f,1,m_world);

    new revolving_platform(3.0f,0.0f,22.0f+rp_shiftx,20.0f,10.0f,m_world);
    new sphere(1.0f,15.f,100.0f,0.01f,36.0f+rp_shiftx,37.2f,1,m_world);
    new revolving_platform(3.0f,0.0f,31.0f+rp_shiftx,19.5f,10.0f,m_world);
    new sphere(1.0f,15.f,100.0f,0.01f,45.0f+rp_shiftx,36.7f,1,m_world);


    new block(15.0f,0.2f,32.0f+rp_shiftx,25.5f,-60,m_world,0, 1.0f,0.05f);




{
//the pulley motor
  float x=50.0f+rp_shiftx;
float y=20.0f;
b=new block(3.0f,0.1f,x,y,0,m_world,1);
b2Body* b1=b->body;
b=new block(3.0f,0.1f,x,y,45,m_world,1);
b2Body* b2=b->body;
b=new block(3.0f,0.1f,x,y,90,m_world,1);
b2Body* b3=b->body;
b=new block(0.001f,0.001f,x,y,0,m_world,0);
b2Body* b4=b->body;
//b=new block(3.0f,0.2f,x,y,5,m_world,0);

//the pendulum
//b=new block(0.2f,5.2f,x,y-5.0f,0,m_world,1);
//b2Body* b5=b->body;
//b->bd.position.Set(0.0f, -52.0f);

// //the bob b6
// b=new block(0.5f,0.5f,x-6.0f,y-10.0f,0,m_world,1,80.0f);
// b2Body* b6=b->body;
//  b6->SetGravityScale(90);
// w=new weld(b2,b6,x, y,m_world);
// r=new rev_j(b6,b3,x, y,m_world);


w=new weld(b1,b2,x, y,m_world);
w=new weld(b2,b3,x, y,m_world);
r=new rev_j(b4,b3,x, y,m_world,-1);
r=new rev_j(b4,b2,x, y,m_world,-1);
r=new rev_j(b4,b1,x, y,m_world,-1);

//test
//b=new block(0.5f,0.5f,x+2.2f,y+5.0f,0,m_world,1);
//b2Body* b7=b->body;

//w=new weld(b6,b5,x, y-10.0f,m_world);


}

new block(0.0f,8.0f,rp_shiftx+58.0f,8.0f,0,m_world,0,0.0f,1.0f);
new block(0.0f,8.0f,rp_shiftx+52.5f,8.0f,0,m_world,0,0.0f,1.0f);



//The pulley system
    {
      float x=10.0f+rp_shiftx+45.3;
      float y=17.0f;
     float x1=24.0f;
      float y1=6.0f;
      float x2=27.0f;
      float y2=7.0f;


open_box* ob=new open_box(x,y-2.0f,10,10,m_world);

b=new block(6.0f,0.2f,x+x1,y+y1,0,m_world,0);
b2Body* b1=b->body;

b=new block(5.0f,0.2f,x+x2,y+y2,0,m_world,1,1000);
b2Body* b2=b->body;
      // The pulley joint
pulley_j* pj=new pulley_j(ob->box1,b2,x,y,x+x2,y+y2-1,x, y+5.0f,x+10.0f, y+5.0f,m_world);


//test
//b=new block(5.0f,0.2f,x,y+5.0f,0,m_world,1,10);

}
Bucket_Water* bw=new Bucket_Water(89.0f+rp_shiftx,23.0f,m_world);


//new CircleQuarter(5,5,5,m_world,90);


new block(20.0f,0.0f,rp_shiftx+80,10,0,m_world,0,1.0f,1.0f);
new block(0.0f,8.0f,rp_shiftx+100.5f,8.0f,0,m_world,0,0.0f,1.0f);


}






sim_t *sim = new sim_t("Dominos", dominos_t::create);
}

// b2BodyDef bodyDef;
// bodyDef.type = b2_dynamicBody;
// bodyDef.position.Set(0.0f, 4.0f);
// b2Body* bodyt = m_world->CreateBody(&bodyDef);
// b2PolygonShape dynamicBox;
// dynamicBox.SetAsBox(1.0f, 1.0f);
// b2FixtureDef fixtureDef;
// fixtureDef.shape = &dynamicBox;
// fixtureDef.density = 1.0f;
// fixtureDef.friction = 0.3f;
// bodyt->CreateFixture(&fixtureDef);
