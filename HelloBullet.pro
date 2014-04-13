#-------------------------------------------------
#
# Project created by QtCreator 2014-04-12T14:13:50
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = HelloBullet
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    b3AlignedAllocator.cpp \
    b3Logging.cpp \
    btAlignedAllocator.cpp \
    btDbvtBroadphase.cpp \
    btDefaultCollisionConfiguration.cpp \
    btCollisionDispatcher.cpp \
    btSequentialImpulseConstraintSolver.cpp \
    btDiscreteDynamicsWorld.cpp \
    btStaticPlaneShape.cpp \
    btConcaveShape.cpp \
    btCollisionShape.cpp \
    btTriangleCallback.cpp \
    btConvexInternalShape.cpp \
    btConvexShape.cpp \
    btBoxShape.cpp \
    btPolyhedralConvexShape.cpp \
    btSphereShape.cpp \
    btCylinderShape.cpp \
    btConeShape.cpp \
    btCapsuleShape.cpp \
    btConvexHullShape.cpp \
    btConvexPointCloudShape.cpp \
    btRigidBody.cpp \
    btTypedConstraint.cpp \
    btDbvt.cpp \
    btOverlappingPairCache.cpp \
    btBroadphaseProxy.cpp \
    btDispatcher.cpp \
    btCollisionAlgorithm.cpp \
    btGjkEpaPenetrationDepthSolver.cpp \
    btVoronoiSimplexSolver.cpp \
    btConvexConvexAlgorithm.cpp \
    btActivatingCollisionAlgorithm.cpp \
    btCollisionObject.cpp \
    btMinkowskiPenetrationDepthSolver.cpp \
    btEmptyCollisionAlgorithm.cpp \
    btConvexConcaveCollisionAlgorithm.cpp \
    btCompoundCollisionAlgorithm.cpp \
    btManifoldResult.cpp \
    btCompoundCompoundCollisionAlgorithm.cpp \
    btConvexPlaneCollisionAlgorithm.cpp \
    btBoxBoxCollisionAlgorithm.cpp \
    btBoxBoxDetector.cpp \
    btSphereSphereCollisionAlgorithm.cpp \
    btSphereTriangleCollisionAlgorithm.cpp \
    SphereTriangleDetector.cpp \
    btPersistentManifold.cpp \
    btQuickprof.cpp \
    btCollisionWorld.cpp \
    btConeTwistConstraint.cpp \
    btGeneric6DofConstraint.cpp \
    btSimulationIslandManager.cpp \
    btGeometryUtil.cpp \
    btVector3.cpp \
    btConvexPolyhedron.cpp \
    btConvexHullComputer.cpp \
    btGjkEpa2.cpp \
    btGjkConvexCast.cpp \
    btConvexCast.cpp \
    btGjkPairDetector.cpp \
    btPolyhedralContactClipping.cpp \
    btSubSimplexConvexCast.cpp \
    btHashedSimplePairCache.cpp \
    btRaycastCallback.cpp \
    btContinuousConvexCollision.cpp \
    btBvhTriangleMeshShape.cpp \
    btUnionFind.cpp \
    btTriangleMeshShape.cpp \
    btStridingMeshInterface.cpp \
    btQuantizedBvh.cpp \
    btOptimizedBvh.cpp

HEADERS += \
    b3AlignedAllocator.h \
    b3Scalar.h \
    b3Logging.h \
    btAlignedAllocator.h \
    btScalar.h \
    btDbvtBroadphase.h \
    btDefaultCollisionConfiguration.h \
    btCollisionConfiguration.h \
    btCollisionDispatcher.h \
    btCollisionCreateFunc.h \
    btSequentialImpulseConstraintSolver.h \
    btDiscreteDynamicsWorld.h \
    btDynamicsWorld.h \
    btStaticPlaneShape.h \
    btConcaveShape.h \
    btCollisionShape.h \
    btTriangleCallback.h \
    btConvexInternalShape.h \
    btConvexShape.h \
    btCollisionMargin.h \
    btTriangleShape.h \
    btBoxShape.h \
    btPolyhedralConvexShape.h \
    btSphereShape.h \
    btCylinderShape.h \
    btConeShape.h \
    btCapsuleShape.h \
    btConvexHullShape.h \
    btConvexPointCloudShape.h \
    btRigidBody.h \
    btTypedConstraint.h \
    btSolverConstraint.h \
    btJacobianEntry.h \
    btSolverBody.h \
    btDbvt.h \
    btOverlappingPairCache.h \
    btBroadphaseInterface.h \
    btBroadphaseProxy.h \
    btOverlappingPairCallback.h \
    btDispatcher.h \
    btCollisionAlgorithm.h \
    btGjkEpaPenetrationDepthSolver.h \
    btConvexPenetrationDepthSolver.h \
    btSimplexSolverInterface.h \
    btVoronoiSimplexSolver.h \
    btConvexConvexAlgorithm.h \
    btActivatingCollisionAlgorithm.h \
    btCollisionObject.h \
    btMinkowskiPenetrationDepthSolver.h \
    btEmptyCollisionAlgorithm.h \
    btConvexConcaveCollisionAlgorithm.h \
    btCompoundCollisionAlgorithm.h \
    btManifoldResult.h \
    btCompoundCompoundCollisionAlgorithm.h \
    btConvexPlaneCollisionAlgorithm.h \
    btBoxBoxCollisionAlgorithm.h \
    btBoxBoxDetector.h \
    btSphereSphereCollisionAlgorithm.h \
    btSphereTriangleCollisionAlgorithm.h \
    SphereTriangleDetector.h \
    btPersistentManifold.h \
    btManifoldPoint.h \
    btQuickprof.h \
    btCollisionWorld.h \
    btConeTwistConstraint.h \
    btGeneric6DofConstraint.h \
    btSimulationIslandManager.h \
    btGeometryUtil.h \
    btVector3.h \
    btMinMax.h \
    btAlignedObjectArray.h \
    btConvexPolyhedron.h \
    btConvexHullComputer.h \
    btGjkEpa2.h \
    btGjkConvexCast.h \
    btConvexCast.h \
    btGjkPairDetector.h \
    btDiscreteCollisionDetectorInterface.h \
    btPointCollector.h \
    btPolyhedralContactClipping.h \
    btSubSimplexConvexCast.h \
    btHashedSimplePairCache.h \
    btRaycastCallback.h \
    btContinuousConvexCollision.h \
    btBvhTriangleMeshShape.h \
    btUnionFind.h \
    btTriangleMeshShape.h \
    btStridingMeshInterface.h \
    btQuantizedBvh.h \
    btOptimizedBvh.h
