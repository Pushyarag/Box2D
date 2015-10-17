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
/**  The is the constructor
 * This is the documentation block for the constructor.
 */

dominos_t::dominos_t()
{
    //Ground
    /*! \var b1
     * \brief pointer to the body ground
     */

    class ground
    {
        /*!  ground object
        * \brief the class to create ground
        */

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
    new ground(m_world);




    class sphere
    {
        /*!  sphere object
        * \brief This is the class to create spheres
        */

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
            if(!gravity)
            {
                sbody->SetGravityScale(-1);
            }
        }
    };

    new sphere(1.0f,5.f,0.0f,0.1f,-52.0f,22.0f,1,m_world);

    class block
    {
        /*!  block object
        * \brief This is the class to create blocks
        */

    public:
        b2Body* body;
        b2BodyDef bd;

        block(float w,float h,float x,float y,float density,int a, b2World* m_world,int s_d=0)
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
            body->CreateFixture(fd);

        }
    };

    new block(10.0f,0.1f,-48.0f,20.0f,1.f,-60,m_world,0);
    new block(1.0f,1.0f,-35.0f,4.5f,0.05f,0,m_world,1);


    //The see-saw system at the bottom
    class see_saw
    {
        /*!  see_saw object
        * \brief This is the class to create see_saw
        */

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

    new see_saw(m_world);

    //The pulley system
    class pulley
    {
        /*!  pulley object
        * \brief This is the class to create pulleys
        */

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
            bd->position.Set(10+x,15+y);
            fd1->density = r_density;///default 34.0f
            b2Body* box2 = m_world->CreateBody(bd);
            box2->CreateFixture(fd1);

            // The pulley joint
            b2PulleyJointDef* myjoint = new b2PulleyJointDef();
            b2Vec2 worldAnchorOnBody1(-10, 15); // Anchor point on body 1 in world axis
            b2Vec2 worldAnchorOnBody2(10, 15); // Anchor point on body 2 in world axis
            b2Vec2 worldAnchorGround1(-10+x, 20+y); // Anchor point for ground 1 in world axis
            b2Vec2 worldAnchorGround2(10+x, 20+y); // Anchor point for ground 2 in world axis
            float32 ratio = 1.0f; // Define ratio
            myjoint->Initialize(box1, box2, worldAnchorGround1+shift, worldAnchorGround2+shift, box1->GetWorldCenter(), box2->GetWorldCenter(), ratio);
            m_world->CreateJoint(myjoint);
        }
    };

    new pulley(-5.0f,5.0f,1.0f,3.0f,m_world);

    //The revolving horizontal platform

    class revolving_platform
    {
    public:

        revolving_platform(float h, float w, float x, float y, b2World *m_world)
        {

            b2PolygonShape shape;
            shape.SetAsBox(2.2f+h, 0.2f+w);

            b2BodyDef bd;
            bd.position.Set(14.0f+x, 14.0f+y);
            bd.type = b2_dynamicBody;
            b2Body* body = m_world->CreateBody(&bd);
            b2FixtureDef *fd = new b2FixtureDef;
            fd->density = 1.f;
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
    new revolving_platform(0.0f,0.0f,-10.0f,10.0f,m_world);
    new sphere(1.0f,5.f,0.0f,0.1f,4.0f,27.2f,1,m_world);
    new sphere(1.0f,5.f,0.0f,0.1f,4.0f,25.2f,0,m_world);



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
