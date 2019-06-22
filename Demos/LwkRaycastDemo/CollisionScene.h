//////////////////////////////////////////////////////////////////////////
// 服务器碰撞检测的实现;
// Create By liwenkun 20171010;
//
//////////////////////////////////////////////////////////////////////////

#ifndef COLLISION_SCENE_H
#define COLLISION_SCENE_H


#include "btBulletCollisionCommon.h"
#include <map>
#include <vector>
#include <string>
#ifndef WIN32
#include <pthread.h>
#endif

//单个碰撞场景;
class CollisionScene
{
public:
	CollisionScene(const std::string & filePath, int instances = 1);

	~CollisionScene();

	bool Load(std::string & errMsg,btCollisionWorld * targetWorld = 0);

	bool RayCast(const btVector3 & source, const btVector3& dest, btVector3 & hitPos, btVector3 &hitNormal,std::string & errMsg, btCollisionWorld * targetWorld = 0);

	bool TryLock();
	bool Lock();
	void Unlock();
private:
	void loadMesh(btCollisionWorld & world, std::ifstream & is);
	void loadBox(btCollisionWorld & world, std::ifstream & is);
	void loadTerrain(btCollisionWorld & world, std::ifstream & is);
private:
	bool m_bLoaded;
	std::string m_filePath;
	std::vector<btCollisionWorld *> m_vCollisionWorlds;
	std::vector< btTriangleIndexVertexArray*> m_vTriangles;
	std::vector<float*> m_vHeightMapData;
	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;
	class btBroadphaseInterface*	m_broadphase;
	class btCollisionDispatcher*	m_dispatcher;
	class btConstraintSolver*	m_solver;
	class btDefaultCollisionConfiguration* m_collisionConfiguration;
	class btCollisionWorld * m_collisionWorld;

#ifndef WIN32
	pthread_spinlock_t     m_spinlock;
#endif
};

//一组碰撞场景;
class CollisionSceneGroup
{
public:
	CollisionSceneGroup(const std::string & filePath,int instance);
	~CollisionSceneGroup();

	bool Load(std::string & errMsg);
	//射线检测;
	bool RayCast(const btVector3 & from, const btVector3& to, btVector3 & hitPos, btVector3 &hitNormal,std::string & errMsg)const;

private:
	std::vector<CollisionScene*> m_scenes;
	volatile int m_lastIndex;
	const int m_instances;
};

//单例;
//要求所有方法调用都是可重入的;
//TODO:优化多线程自旋锁等待和内存占用之间取舍;
class CollisionSceneMgr
{
public:
	static CollisionSceneMgr & GetInstance();

	CollisionSceneMgr();
	~CollisionSceneMgr();

	void CleanUp();

	bool addScene(int protoId, const std::string & filePath,int instance,bool loadNow, std::string & errmsg);

	//射线检测;
	bool RayCast(int protoId, const btVector3 & from, const btVector3& to, btVector3 & hitPos, btVector3 &hitNormal,std::string & errMsg)const;

private:
	std::map<int,CollisionSceneGroup*> m_mapProtoId2SceneGroup;  //不拥有这些指针;
};

#endif //CONCAVE_RAYCAST_DEMO_H

