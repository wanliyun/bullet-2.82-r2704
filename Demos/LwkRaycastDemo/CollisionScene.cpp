//////////////////////////////////////////////////////////////////////////
// 服务器碰撞检测的实现;
// Create By liwenkun 20171010;
//
// 考虑：
// 1.skynet多线程射线检测;
// 2.每个分配一个碰撞场景会造成极大的内存浪费;
//
// 设计:
// 1.对于特定碰撞场景，rayCast用自旋锁保护;
// 2.为每个场景ProtoId开辟N个碰撞场景,减少自旋锁征用;
//
//////////////////////////////////////////////////////////////////////////

#include "CollisionScene.h"
#include "BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"
#include <fstream>
#include <string>
#include <cassert>


static void localCreateRigidBody(btCollisionWorld & world, btTransform& startTransform,btCollisionShape* shape);

CollisionScene::CollisionScene(const std::string & filePath, int instances /*= 1*/):
	m_bLoaded(false),
	m_filePath(filePath),
	m_broadphase(0),
	m_dispatcher(0),
	m_collisionConfiguration(0),
	m_collisionWorld(0)
{
#ifndef WIN32
	 pthread_spin_init(&m_spinlock,PTHREAD_PROCESS_PRIVATE);
#endif
}

CollisionScene::~CollisionScene()
{
	if (!m_collisionWorld)
	{
		return;
	}

	for (int i = m_collisionWorld->getNumCollisionObjects() - 1; i >= 0; i--)
	{
		btCollisionObject* obj = m_collisionWorld->getCollisionObjectArray()[i];
		m_collisionWorld->removeCollisionObject(obj);
		delete obj;
	}

	//delete collision shapes
	for (int j = 0; j < m_collisionShapes.size(); j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}
	m_collisionShapes.clear();

	for (size_t i = 0; i < m_vHeightMapData.size(); ++i)
	{
		float * data = m_vHeightMapData[i];
		delete[] data;
	}
	m_vHeightMapData.clear();

	for (size_t i = 0; i < m_vTriangles.size(); ++i)
	{
		btTriangleIndexVertexArray * p = m_vTriangles[i];
		delete p;
	}
	m_vTriangles.clear();

	//delete dynamics world
	delete m_collisionWorld;

	//delete broadphase
	delete m_broadphase;

	//delete dispatcher
	delete m_dispatcher;

	delete m_collisionConfiguration;

#ifndef WIN32
	pthread_spin_destroy(&m_spinlock);
#endif
}

bool CollisionScene::Load(std::string & errMsg,btCollisionWorld * targetWorld /*= 0*/)
{
	if(!targetWorld)
	{
		m_collisionConfiguration = new btDefaultCollisionConfiguration();
		m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);

		//TODO:裁切;
		btVector3 worldMin(0,-1000,0);
		btVector3 worldMax(5120, 5120, 5120);
		m_broadphase = new btAxisSweep3(worldMin,worldMax);
		m_collisionWorld = new btCollisionWorld(m_dispatcher, m_broadphase, m_collisionConfiguration);
		targetWorld = m_collisionWorld;
		m_bLoaded = true;
	}

	std::ifstream is(m_filePath.c_str(),std::ios::binary);
	if(!is)
	{
		errMsg = "Failed to load file";
		errMsg += m_filePath;
		return false;
	}

	int version = 0;
	int meshCount = 0;
	int boxCount = 0;
	int terrainCount = 0;
	is.read((char*)&version,sizeof(version));
	if (version == 3)
	{
		is.read((char*)&terrainCount, sizeof(terrainCount));
		is.read((char*)&boxCount, sizeof(boxCount));
		is.read((char*)&meshCount, sizeof(meshCount));
	}
	else
	{
		errMsg = "Unsupported collision file version";
		errMsg += m_filePath;
		return false;
	}

	//printf("version=%d meshCount=%d boxCount=%d",version,meshCount,boxCount);
	for(int i = 0; i < terrainCount; ++i)
	{
		this->loadTerrain(*targetWorld, is);
	}
	for(int i = 0;i<boxCount;++i)
	{
		this->loadBox(*targetWorld,is);
	}
	for (int i = 0; i < meshCount; ++i)
	{
		this->loadMesh(*targetWorld, is);
	}

	return true;
}

bool CollisionScene::RayCast(const btVector3 & from, const btVector3& to, btVector3 & hitPos, btVector3 &hitNormal,std::string & errMsg, btCollisionWorld * targetWorld /*= 0*/)
{
	if(!m_bLoaded)
	{
		std::string errMsg;
		Load(errMsg);
	}

	if(!m_collisionWorld)
	{
		errMsg= "load file failed!";
		return false;
	}

	btCollisionWorld::ClosestRayResultCallback cb(from, to);


	m_collisionWorld->rayTest (from, to, cb);

	if (cb.hasHit())
	{
		hitPos = cb.m_hitPointWorld;
		hitNormal = cb.m_hitNormalWorld;
		hitNormal.normalize ();
		return true;
	}

	return false;
}


bool CollisionScene::TryLock()
{
#ifndef WIN32
	return 0 == pthread_spin_trylock(&m_spinlock);
#else
	return true;
#endif
}


bool CollisionScene::Lock()
{
#ifndef WIN32
	return 0 == pthread_spin_lock(&m_spinlock);
#else
	return true;
#endif
}

void CollisionScene::Unlock()
{
#ifndef WIN32
	pthread_spin_unlock(&m_spinlock);
#endif
}

static void localCreateRigidBody(btCollisionWorld & world, btTransform& startTransform,btCollisionShape* shape)
{
	btCollisionObject* body = new btCollisionObject();	
	body->setCollisionShape(shape);
	body->setCollisionFlags( body->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT );
	body->setWorldTransform(startTransform);
	
	world.addCollisionObject(body);
}

inline static void ReadVec3(std::ifstream & is, btVector3 & v)
{
	is.read((char*)&v, sizeof(float) * 3);
}

inline static void ReadQuat(std::ifstream & is, btQuaternion & v)
{
	is.read((char*)&v, sizeof(float) * 4);
}

//////////////////////////////////////////////////////////////////////////
CollisionSceneGroup::CollisionSceneGroup(const std::string & filePath,int instance):
	m_lastIndex(0),
	m_instances(instance)
{
	m_scenes.resize(instance);
	for(int i = 0;i<instance;++i)
	{
		m_scenes[i] = new CollisionScene(filePath);
	}
}

CollisionSceneGroup::~CollisionSceneGroup()
{
	for(std::vector<CollisionScene*>::iterator iter = m_scenes.begin();iter!= m_scenes.end();++iter)
	{
		delete *iter;
	}
}

bool CollisionSceneGroup::Load(std::string & errMsg)
{
	bool bRet = true;
	for(std::vector<CollisionScene*>::iterator iter = m_scenes.begin();iter!= m_scenes.end();++iter)
	{
		CollisionScene * pScene = *iter;
		pScene->Lock();
		bRet &= pScene->Load(errMsg);
		pScene->Unlock();
	}
	return bRet;
}

bool CollisionSceneGroup::RayCast(const btVector3 & from, const btVector3& to, btVector3 & hitPos, btVector3 &hitNormal,std::string & errMsg) const
{
	int n = (m_lastIndex+1) % m_instances;
	bool bRet = false;
	for(;;n=(n+1) % m_instances)
	{
		CollisionScene * pScene = m_scenes[n];
		if(pScene->TryLock())
		{
			bRet = pScene->RayCast(from,to,hitPos,hitNormal,errMsg);
			pScene->Unlock();
			break;
		}
	}
	return bRet;
}

void CollisionScene::loadTerrain(btCollisionWorld & world, std::ifstream & is)
{
	btVector3 trans; ReadVec3(is, trans);
	int stickWidth;
	int stickHeight;
	is.read((char*)&stickWidth, sizeof(int));
	is.read((char*)&stickHeight, sizeof(int));
	btVector3 scale;  ReadVec3(is, scale);
	float minHeight, maxHeight;
	is.read((char*)&minHeight, sizeof(float));
	is.read((char*)&maxHeight, sizeof(float));
	int upAix;
	is.read((char*)&upAix, sizeof(int));

	int pixSize = stickWidth * stickHeight;
	float * heightfieldData = new float[pixSize];
	is.read((char*)heightfieldData, pixSize * sizeof(float));
	m_vHeightMapData.push_back(heightfieldData);

	btHeightfieldTerrainShape* heightFieldShape = new btHeightfieldTerrainShape(stickWidth, stickHeight, heightfieldData, 1, minHeight, maxHeight, upAix, PHY_FLOAT, false);

	heightFieldShape->setUseDiamondSubdivision(true);

	heightFieldShape->setLocalScaling(scale);
	btVector3 mmin, mmax;
	heightFieldShape->getAabb(btTransform::getIdentity(), mmin, mmax);

	mmax.setZ(-mmax.getZ());
	trans += mmax;
	btTransform startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(trans);

	heightFieldShape->getAabb(startTransform, mmin, mmax);

	localCreateRigidBody(world, startTransform, heightFieldShape);
}
void CollisionScene::loadMesh(btCollisionWorld & world, std::ifstream & is)
{
	using namespace std;

	const int vertStride = sizeof(btVector3);
	const int indexStride = 3 * sizeof(int);

	int instanceCount = 0;
	is.read((char*)&instanceCount, sizeof(int));
	if (instanceCount <= 0)
	{
		return;
	}

	int vertexCount, indexCount;
	is.read((char*)&vertexCount, sizeof(int));
	is.read((char*)&indexCount, sizeof(int));

	//printf("loadMesh vertexCount=%d,indexCount=%d instanceCount=%d \n", vertexCount , indexCount, instanceCount);
	btVector3 * pVerts = new btVector3[vertexCount];
	is.read((char*)pVerts, sizeof(btVector3) * vertexCount);

	int * pIndices = new int[indexCount];
	is.read((char*)pIndices, sizeof(int) * indexCount);

	btVector3 trans, scale;
	btQuaternion rot;
	btTransform startTransform;
	for (int i = 0; i < instanceCount; ++i)
	{
		btVector3 * pVertsCpy = new btVector3[vertexCount];
		memcpy(pVertsCpy, pVerts, sizeof(pVerts[0]) * vertexCount);

		int * pIndicesCpy = new int[indexCount];
		memcpy(pIndicesCpy, pIndices, sizeof(pIndices[0]) * indexCount);

		btTriangleIndexVertexArray * pArray = new btTriangleIndexVertexArray(indexCount / 3,
			pIndicesCpy,
			indexStride,
			vertexCount,
			(btScalar*)&pVertsCpy[0].x(), vertStride);

		ReadVec3(is, trans);
		ReadVec3(is, scale);
		ReadQuat(is, rot);

		startTransform.setIdentity();
		btTransform startTransform(rot, trans);

		btBvhTriangleMeshShape * pShredShape = new btBvhTriangleMeshShape(pArray, true);
		pShredShape->setLocalScaling(scale);

		localCreateRigidBody(world, startTransform, pShredShape);
	}

	delete[] pVerts;
	delete[] pIndices;
}

void CollisionScene::loadBox(btCollisionWorld & world, std::ifstream & is)
{
	using namespace std;
	btVector3 t; ReadVec3(is, t);
	btVector3 s; ReadVec3(is, s);
	btQuaternion q; ReadQuat(is, q);
	
	btBoxShape * pBox = new btBoxShape(s / 2 );
	btTransform	startTransform(q, t);
	localCreateRigidBody(world, startTransform, pBox);
}

//////////////////////////////////////////////////////////////////////////
CollisionSceneMgr & CollisionSceneMgr::GetInstance()
{
	static CollisionSceneMgr inst;
	return inst;
}

CollisionSceneMgr::CollisionSceneMgr()
{

}

CollisionSceneMgr::~CollisionSceneMgr()
{
	CleanUp();
}


void CollisionSceneMgr::CleanUp()
{
	for(std::map<int,CollisionSceneGroup*>::iterator iter = m_mapProtoId2SceneGroup.begin();iter!= m_mapProtoId2SceneGroup.end();++iter)
	{
		delete iter->second;
	}
	m_mapProtoId2SceneGroup.clear();
}

bool CollisionSceneMgr::addScene(int protoId, const std::string & filePath,int instance,bool loadNow,std::string & errmsg)
{
	if(m_mapProtoId2SceneGroup.find(protoId) != m_mapProtoId2SceneGroup.end())
	{
		return false;
	}

	CollisionSceneGroup *pGroup = new CollisionSceneGroup(filePath, instance);
	if(loadNow)
	{
		if(!pGroup->Load(errmsg))
		{
			delete pGroup;
			return false;
		}
	}
	m_mapProtoId2SceneGroup[protoId] = pGroup;
	return true;
}

bool CollisionSceneMgr::RayCast(int protoId, const btVector3 & from, const btVector3& to, btVector3 & hitPos, btVector3 &hitNormal,std::string & errMsg)const
{
	std::map<int,CollisionSceneGroup* >::const_iterator iter_find = m_mapProtoId2SceneGroup.find(protoId);
	if( iter_find == m_mapProtoId2SceneGroup.end())
	{
		errMsg = "Can not find Scene name! please check protoId";
		return false;
	}

	return iter_find->second->RayCast(from, to, hitPos,hitNormal,errMsg);
}

