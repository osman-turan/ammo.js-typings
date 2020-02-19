declare function Ammo(): Promise<void>;

declare namespace Ammo {
  type VoidPtr = number;

  interface btIDebugDraw {
    drawLine(from: btVector3, to: btVector3, color: btVector3): void;
    drawContactPoint(
      pointOnB: btVector3,
      normalOnB: btVector3,
      distance: number,
      lifeTime: number,
      color: btVector3
    ): void;
    reportErrorWarning(warningString: string): void;
    draw3dText(location: btVector3, textString: string): void;
    setDebugMode(debugMode: number): void;
    getDebugMode(): number;
  }

  class DebugDrawer implements btIDebugDraw {
    constructor();
    drawLine(from: btVector3, to: btVector3, color: btVector3): void;
    drawContactPoint(
      pointOnB: btVector3,
      normalOnB: btVector3,
      distance: number,
      lifeTime: number,
      color: btVector3
    ): void;
    reportErrorWarning(warningString: string): void;
    draw3dText(location: btVector3, textString: string): void;
    setDebugMode(debugMode: number): void;
    getDebugMode(): number;
  }

  class btVector3 {
    constructor();
    constructor(x: number, y: number, z: number);
    length(): number;
    x(): number;
    y(): number;
    z(): number;
    setX(x: number): void;
    setY(y: number): void;
    setZ(z: number): void;
    setValue(x: number, y: number, z: number): void;
    normalize(): void;
    rotate(wAxis: btVector3, angle: number): btVector3;
    dot(v: btVector3): number;
    op_mul(x: number): btVector3;
    op_add(v: btVector3): btVector3;
    op_sub(v: btVector3): btVector3;
  }

  class btVector4 extends btVector3 {
    constructor();
    constructor(x: number, y: number, z: number, w: number);
    w(): number;
    setValue(x: number, y: number, z: number): void;
    setValue(x: number, y: number, z: number, w: number): void;
  }

  class btQuadWord {
    x(): number;
    y(): number;
    z(): number;
    w(): number;
    setX(x: number): void;
    setY(y: number): void;
    setZ(z: number): void;
    setW(w: number): void;
  }

  class btQuaternion extends btQuadWord {
    constructor(x: number, y: number, z: number, w: number);
    setValue(x: number, y: number, z: number, w: number): void;
    setEulerZYX(z: number, y: number, x: number): void;
    setRotation(axis: btVector3, angle: number): void;
    normalize(): void;
    length2(): number;
    length(): number;
    dot(q: btQuaternion): number;
    normalized(): btQuaternion;
    getAxis(): btVector3;
    inverse(): btQuaternion;
    getAngle(): number;
    getAngleShortestPath(): number;
    angle(q: btQuaternion): number;
    angleShortestPath(q: btQuaternion): number;
    op_add(q: btQuaternion): btQuaternion;
    op_sub(q: btQuaternion): btQuaternion;
    op_mul(s: number): btQuaternion;
    op_mulq(q: btQuaternion): btQuaternion;
    op_div(s: number): btQuaternion;
  }

  class btMatrix3x3 {
    setEulerZYX(ex: number, ey: number, ez: number): void;
    getRotation(q: btQuaternion): void;
    getRow(y: number): btVector3;
  }

  class btTransform {
    constructor();
    constructor(q: btQuaternion, v: btVector3);
    setIdentity(): void;
    setOrigin(origin: btVector3): void;
    setRotation(rotation: btQuaternion): void;
    getOrigin(): btVector3;
    getRotation(): btQuaternion;
    getBasis(): btMatrix3x3;
    setFromOpenGLMatrix(m: number[]): void;
    inverse(): btTransform;
    op_mul(t: btTransform): btTransform;
  }

  class btMotionState {
    getWorldTransform(worldTrans: btTransform): void;
    setWorldTransform(worldTrans: btTransform): void;
  }

  class btDefaultMotionState extends btMotionState {
    constructor(startTrans?: btTransform, centerOfMassOffset?: btTransform);
    m_graphicsWorldTrans: btTransform;
  }

  class btCollisionObject {
    setAnisotropicFriction(
      anisotropicFriction: btVector3,
      frictionMode: number
    ): void;
    getCollisionShape(): btCollisionShape;
    setContactProcessingThreshold(contactProcessingThreshold: number): void;
    setActivationState(newState: number): void;
    forceActivationState(newState: number): void;
    activate(forceActivation?: boolean): void;
    isActive(): boolean;
    isKinematicObject(): boolean;
    isStaticObject(): boolean;
    isStaticOrKinematicObject(): boolean;
    getRestitution(): number;
    getFriction(): number;
    getRollingFriction(): number;
    setRestitution(rest: number): void;
    setFriction(frict: number): void;
    setRollingFriction(frict: number): void;
    getWorldTransform(): btTransform;
    getCollisionFlags(): number;
    setCollisionFlags(flags: number): void;
    setWorldTransform(worldTrans: btTransform): void;
    setCollisionShape(collisionShape: btCollisionShape): void;
    setCcdMotionThreshold(ccdMotionThreshold: number): void;
    setCcdSweptSphereRadius(radius: number): void;
    getUserIndex(): number;
    setUserIndex(index: number): void;
    getUserPointer(): VoidPtr;
    setUserPointer(userPointer: VoidPtr): void;
    getBroadphaseHandle(): btBroadphaseProxy;
  }

  class btCollisionObjectWrapper {
    getWorldTransform(): btTransform;
    getCollisionObject(): btCollisionObject;
    getCollisionShape(): btCollisionShape;
  }

  class RayResultCallback {
    hasHit(): boolean;
    m_collisionFilterGroup: number;
    m_collisionFilterMask: number;
    m_closestHitFraction: number;
    m_collisionObject: btCollisionObject;
  }

  class ClosestRayResultCallback extends RayResultCallback {
    constructor(from: btVector3, to: btVector3);
    m_rayFromWorld: btVector3;
    m_rayToWorld: btVector3;
    m_hitNormalWorld: btVector3;
    m_hitPointWorld: btVector3;
  }

  class btConstCollisionObjectArray {
    size(): number;
    at(n: number): btCollisionObject;
  }

  class btScalarArray {
    size(): number;
    at(n: number): number;
  }

  class AllHitsRayResultCallback extends RayResultCallback {
    constructor(from: btVector3, to: btVector3);
    m_collisionObjects: btConstCollisionObjectArray;
    m_rayFromWorld: btVector3;
    m_rayToWorld: btVector3;
    m_hitNormalWorld: btVector3Array;
    m_hitPointWorld: btVector3Array;
    m_hitFractions: btScalarArray;
  }

  class btManifoldPoint {
    getPositionWorldOnA(): btVector3;
    getPositionWorldOnB(): btVector3;
    getAppliedImpulse(): number;
    getDistance(): number;
    m_localPointA: btVector3;
    m_localPointB: btVector3;
    m_positionWorldOnB: btVector3;
    m_positionWorldOnA: btVector3;
    m_normalWorldOnB: btVector3;
    m_userPersistentData: any;
  }

  class ContactResultCallback {
    addSingleResult(
      cp: btManifoldPoint,
      colObj0Wrap: btCollisionObjectWrapper,
      partId0: number,
      index0: number,
      colObj1Wrap: btCollisionObjectWrapper,
      partId1: number,
      index1: number
    ): number;
  }

  class ConcreteContactResultCallback extends ContactResultCallback {
    constructor();
    addSingleResult(
      cp: btManifoldPoint,
      colObj0Wrap: btCollisionObjectWrapper,
      partId0: number,
      index0: number,
      colObj1Wrap: btCollisionObjectWrapper,
      partId1: number,
      index1: number
    ): number;
  }

  class LocalShapeInfo {
    m_shapePart: number;
    m_triangleIndex: number;
  }

  class LocalConvexResult {
    constructor(
      hitCollisionObject: btCollisionObject,
      localShapeInfo: LocalShapeInfo,
      hitNormalLocal: btVector3,
      hitPointLocal: btVector3,
      hitFraction: number
    );
    m_hitCollisionObject: btCollisionObject;
    m_localShapeInfo: LocalShapeInfo;
    m_hitNormalLocal: btVector3;
    m_hitPointLocal: btVector3;
    m_hitFraction: number;
  }

  class ConvexResultCallback {
    hasHit(): boolean;
    m_collisionFilterGroup: number;
    m_collisionFilterMask: number;
    m_closestHitFraction: number;
  }

  class ClosestConvexResultCallback extends ConvexResultCallback {
    constructor(convexFromWorld: btVector3, convexToWorld: btVector3);
    m_convexFromWorld: btVector3;
    m_convexToWorld: btVector3;
    m_hitNormalWorld: btVector3;
    m_hitPointWorld: btVector3;
  }

  class btCollisionShape {
    setLocalScaling(scaling: btVector3): void;
    getLocalScaling(): btVector3;
    calculateLocalInertia(mass: number, inertia: btVector3): void;
    setMargin(margin: number): void;
    getMargin(): number;
  }

  class btConvexShape extends btCollisionShape {}

  class btConvexTriangleMeshShape extends btConvexShape {
    constructor(meshInterface: btStridingMeshInterface, calcAabb?: boolean);
  }

  class btBoxShape extends btCollisionShape {
    constructor(boxHalfExtents: btVector3);
    setMargin(margin: number): void;
    getMargin(): number;
  }

  class btCapsuleShape extends btCollisionShape {
    constructor(radius: number, height: number);
    setMargin(margin: number): void;
    getMargin(): number;
    getUpAxis(): number;
    getRadius(): number;
    getHalfHeight(): number;
  }

  class btCapsuleShapeX extends btCapsuleShape {
    constructor(radius: number, height: number);
    setMargin(margin: number): void;
    getMargin(): number;
  }

  class btCapsuleShapeZ extends btCapsuleShape {
    constructor(radius: number, height: number);
    setMargin(margin: number): void;
    getMargin(): number;
  }

  class btCylinderShape extends btCollisionShape {
    constructor(halfExtents: btVector3);
    setMargin(margin: number): void;
    getMargin(): number;
  }

  class btCylinderShapeX extends btCylinderShape {
    constructor(halfExtents: btVector3);
    setMargin(margin: number): void;
    getMargin(): number;
  }

  class btCylinderShapeZ extends btCylinderShape {
    constructor(halfExtents: btVector3);
    setMargin(margin: number): void;
    getMargin(): number;
  }

  class btSphereShape extends btCollisionShape {
    constructor(radius: number);
    setMargin(margin: number): void;
    getMargin(): number;
  }

  class btMultiSphereShape extends btCollisionShape {
    constructor(positions: btVector3, radii: number[], numPoints: number);
  }

  class btConeShape extends btCollisionShape {
    constructor(radius: number, height: number);
  }

  class btConeShapeX extends btConeShape {
    constructor(radius: number, height: number);
  }

  class btConeShapeZ extends btConeShape {
    constructor(radius: number, height: number);
  }

  class btIntArray {
    size(): number;
    at(n: number): number;
  }

  class btFace {
    m_indices: btIntArray;
    m_plane: number[];
  }

  class btVector3Array {
    size(): number;
    at(n: number): btVector3;
  }

  class btFaceArray {
    size(): number;
    at(n: number): btFace;
  }

  class btConvexPolyhedron {
    m_vertices: btVector3Array;
    m_faces: btFaceArray;
  }

  class btConvexHullShape extends btCollisionShape {
    constructor(points?: number[], numPoints?: number);
    addPoint(point: btVector3, recalculateLocalAABB?: boolean): void;
    setMargin(margin: number): void;
    getMargin(): number;
    getNumVertices(): number;
    initializePolyhedralFeatures(shiftVerticesByMargin: number): boolean;
    recalcLocalAabb(): void;
    getConvexPolyhedron(): btConvexPolyhedron;
  }

  class btShapeHull {
    constructor(shape: btConvexShape);
    buildHull(margin: number): boolean;
    numVertices(): number;
    getVertexPointer(): btVector3;
  }

  class btCompoundShape extends btCollisionShape {
    constructor(enableDynamicAabbTree?: boolean);
    addChildShape(localTransform: btTransform, shape: btCollisionShape): void;
    removeChildShape(shape: btCollisionShape): void;
    removeChildShapeByIndex(childShapeindex: number): void;
    getNumChildShapes(): number;
    getChildShape(index: number): btCollisionShape;
    updateChildTransform(
      childIndex: number,
      newChildTransform: btTransform,
      shouldRecalculateLocalAabb?: boolean
    ): void;
    setMargin(margin: number): void;
    getMargin(): number;
  }

  class btStridingMeshInterface {
    setScaling(scaling: btVector3): void;
  }

  class btIndexedMesh {
    m_numTriangles: number;
  }

  class btIndexedMeshArray {
    size(): number;
    at(n: number): btIndexedMesh;
  }

  class btTriangleMesh extends btStridingMeshInterface {
    constructor(use32bitIndices?: boolean, use4componentVertices?: boolean);
    addTriangle(
      vertex0: btVector3,
      vertex1: btVector3,
      vertex2: btVector3,
      removeDuplicateVertices?: boolean
    ): void;
    findOrAddVertex(
      vertex: btVector3,
      removeDuplicateVertices: boolean
    ): number;
    addIndex(index: number): void;
    getIndexedMeshArray(): btIndexedMeshArray;
  }

  enum PHY_ScalarType {
    PHY_FLOAT,
    PHY_DOUBLE,
    PHY_INTEGER,
    PHY_SHORT,
    PHY_FIXEDPOINT88,
    PHY_UCHAR
  }

  class btConcaveShape extends btCollisionShape {}

  class btEmptyShape extends btConcaveShape {
    constructor();
  }

  class btStaticPlaneShape extends btConcaveShape {
    constructor(planeNormal: btVector3, planeConstant: number);
  }

  class btTriangleMeshShape extends btConcaveShape {}

  class btBvhTriangleMeshShape extends btTriangleMeshShape {
    constructor(
      meshInterface: btStridingMeshInterface,
      useQuantizedAabbCompression: boolean,
      buildBvh?: boolean
    );
  }

  class btHeightfieldTerrainShape extends btConcaveShape {
    constructor(
      heightStickWidth: number,
      heightStickLength: number,
      heightfieldData: VoidPtr,
      heightScale: number,
      minHeight: number,
      maxHeight: number,
      upAxis: number,
      hdt: PHY_ScalarType,
      flipQuadEdges: boolean
    );
    setMargin(margin: number): void;
    getMargin(): number;
  }

  class btDefaultCollisionConstructionInfo {
    constructor();
  }

  class btDefaultCollisionConfiguration {
    constructor(info?: btDefaultCollisionConstructionInfo);
  }

  class btPersistentManifold {
    constructor();
    getBody0(): btCollisionObject;
    getBody1(): btCollisionObject;
    getNumContacts(): number;
    getContactPoint(index: number): btManifoldPoint;
  }

  class btDispatcher {
    getNumManifolds(): number;
    getManifoldByIndexInternal(index: number): btPersistentManifold;
  }

  class btCollisionDispatcher extends btDispatcher {
    constructor(conf: btDefaultCollisionConfiguration);
  }

  class btOverlappingPairCallback {}

  class btOverlappingPairCache {
    setInternalGhostPairCallback(
      ghostPairCallback: btOverlappingPairCallback
    ): void;
    getNumOverlappingPairs(): number;
  }

  class btAxisSweep3 {
    constructor(
      worldAabbMin: btVector3,
      worldAabbMax: btVector3,
      maxHandles?: number,
      pairCache?: btOverlappingPairCache,
      disableRaycastAccelerator?: boolean
    );
  }

  class btBroadphaseInterface {
    getOverlappingPairCache(): btOverlappingPairCache;
  }

  class btCollisionConfiguration {}

  class btDbvtBroadphase {
    constructor();
  }

  class btBroadphaseProxy {
    m_collisionFilterGroup: number;
    m_collisionFilterMask: number;
  }

  class btRigidBodyConstructionInfo {
    constructor(
      mass: number,
      motionState: btMotionState,
      collisionShape: btCollisionShape,
      localInertia?: btVector3
    );
    m_linearDamping: number;
    m_angularDamping: number;
    m_friction: number;
    m_rollingFriction: number;
    m_restitution: number;
    m_linearSleepingThreshold: number;
    m_angularSleepingThreshold: number;
    m_additionalDamping: boolean;
    m_additionalDampingFactor: number;
    m_additionalLinearDampingThresholdSqr: number;
    m_additionalAngularDampingThresholdSqr: number;
    m_additionalAngularDampingFactor: number;
  }

  class btRigidBody extends btCollisionObject {
    constructor(constructionInfo: btRigidBodyConstructionInfo);
    getCenterOfMassTransform(): btTransform;
    setCenterOfMassTransform(xform: btTransform): void;
    setSleepingThresholds(linear: number, angular: number): void;
    getLinearDamping(): number;
    getAngularDamping(): number;
    setDamping(lin_damping: number, ang_damping: number): void;
    setMassProps(mass: number, inertia: btVector3): void;
    getLinearFactor(): btVector3;
    setLinearFactor(linearFactor: btVector3): void;
    applyTorque(torque: btVector3): void;
    applyLocalTorque(torque: btVector3): void;
    applyForce(force: btVector3, rel_pos: btVector3): void;
    applyCentralForce(force: btVector3): void;
    applyCentralLocalForce(force: btVector3): void;
    applyTorqueImpulse(torque: btVector3): void;
    applyImpulse(impulse: btVector3, rel_pos: btVector3): void;
    applyCentralImpulse(impulse: btVector3): void;
    updateInertiaTensor(): void;
    getLinearVelocity(): btVector3;
    getAngularVelocity(): btVector3;
    setLinearVelocity(lin_vel: btVector3): void;
    setAngularVelocity(ang_vel: btVector3): void;
    getMotionState(): btMotionState;
    setMotionState(motionState: btMotionState): void;
    getAngularFactor(): btVector3;
    setAngularFactor(angularFactor: btVector3): void;
    upcast(colObj: btCollisionObject): btRigidBody;
    getAabb(aabbMin: btVector3, aabbMax: btVector3): void;
    applyGravity(): void;
    getGravity(): btVector3;
    setGravity(acceleration: btVector3): void;
    getBroadphaseProxy(): btBroadphaseProxy;
  }

  class btConstraintSetting {
    constructor();
    m_tau: number;
    m_damping: number;
    m_impulseClamp: number;
  }

  class btTypedConstraint {
    enableFeedback(needsFeedback: boolean): void;
    getBreakingImpulseThreshold(): number;
    setBreakingImpulseThreshold(threshold: number): void;
    getParam(num: number, axis: number): number;
    setParam(num: number, value: number, axis: number): void;
  }

  enum btConstraintParams {
    BT_CONSTRAINT_ERP,
    BT_CONSTRAINT_STOP_ERP,
    BT_CONSTRAINT_CFM,
    BT_CONSTRAINT_STOP_CFM
  }

  class btPoint2PointConstraint extends btTypedConstraint {
    constructor(
      rbA: btRigidBody,
      rbB: btRigidBody,
      pivotInA: btVector3,
      pivotInB: btVector3
    );
    constructor(rbA: btRigidBody, pivotInA: btVector3);
    setPivotA(pivotA: btVector3): void;
    setPivotB(pivotB: btVector3): void;
    getPivotInA(): btVector3;
    getPivotInB(): btVector3;
    m_setting: btConstraintSetting;
  }

  class btGeneric6DofConstraint extends btTypedConstraint {
    constructor(
      rbA: btRigidBody,
      rbB: btRigidBody,
      frameInA: btTransform,
      frameInB: btTransform,
      useLinearFrameReferenceFrameA: boolean
    );
    constructor(
      rbB: btRigidBody,
      frameInB: btTransform,
      useLinearFrameReferenceFrameB: boolean
    );
    setLinearLowerLimit(linearLower: btVector3): void;
    setLinearUpperLimit(linearUpper: btVector3): void;
    setAngularLowerLimit(angularLower: btVector3): void;
    setAngularUpperLimit(angularUpper: btVector3): void;
    getFrameOffsetA(): btTransform;
  }

  class btGeneric6DofSpringConstraint extends btGeneric6DofConstraint {
    constructor(
      rbA: btRigidBody,
      rbB: btRigidBody,
      frameInA: btTransform,
      frameInB: btTransform,
      useLinearFrameReferenceFrameA: boolean
    );
    constructor(
      rbB: btRigidBody,
      frameInB: btTransform,
      useLinearFrameReferenceFrameB: boolean
    );
    enableSpring(index: number, onOff: boolean): void;
    setStiffness(index: number, stiffness: number): void;
    setDamping(index: number, damping: number): void;
    setEquilibriumPoint(index: number, val: number): void;
    setEquilibriumPoint(index: number): void;
    setEquilibriumPoint(): void;
  }

  class btSequentialImpulseConstraintSolver {
    constructor();
  }

  class btConeTwistConstraint extends btTypedConstraint {
    constructor(
      rbA: btRigidBody,
      rbB: btRigidBody,
      rbAFrame: btTransform,
      rbBFrame: btTransform
    );
    constructor(rbA: btRigidBody, rbAFrame: btTransform);
    setLimit(limitIndex: number, limitValue: number): void;
    setAngularOnly(angularOnly: boolean): void;
    setDamping(damping: number): void;
    enableMotor(b: boolean): void;
    setMaxMotorImpulse(maxMotorImpulse: number): void;
    setMaxMotorImpulseNormalized(maxMotorImpulse: number): void;
    setMotorTarget(q: btQuaternion): void;
    setMotorTargetInConstraintSpace(q: btQuaternion): void;
  }

  class btHingeConstraint extends btTypedConstraint {
    constructor(
      rbA: btRigidBody,
      rbB: btRigidBody,
      pivotInA: btVector3,
      pivotInB: btVector3,
      axisInA: btVector3,
      axisInB: btVector3,
      useReferenceFrameA?: boolean
    );
    constructor(
      rbA: btRigidBody,
      rbB: btRigidBody,
      rbAFrame: btTransform,
      rbBFrame: btTransform,
      useReferenceFrameA?: boolean
    );
    constructor(
      rbA: btRigidBody,
      rbAFrame: btTransform,
      useReferenceFrameA?: boolean
    );
    setLimit(
      low: number,
      high: number,
      softness: number,
      biasFactor: number,
      relaxationFactor?: number
    ): void;
    enableAngularMotor(
      enableMotor: boolean,
      targetVelocity: number,
      maxMotorImpulse: number
    ): void;
    setAngularOnly(angularOnly: boolean): void;
    enableMotor(enableMotor: boolean): void;
    setMaxMotorImpulse(maxMotorImpulse: number): void;
    setMotorTarget(targetAngle: number, dt: number): void;
  }

  class btSliderConstraint extends btTypedConstraint {
    constructor(
      rbA: btRigidBody,
      rbB: btRigidBody,
      frameInA: btTransform,
      frameInB: btTransform,
      useLinearReferenceFrameA: boolean
    );
    constructor(
      rbB: btRigidBody,
      frameInB: btTransform,
      useLinearReferenceFrameA: boolean
    );
    setLowerLinLimit(lowerLimit: number): void;
    setUpperLinLimit(upperLimit: number): void;
    setLowerAngLimit(lowerAngLimit: number): void;
    setUpperAngLimit(upperAngLimit: number): void;
  }

  class btFixedConstraint extends btTypedConstraint {
    constructor(
      rbA: btRigidBody,
      rbB: btRigidBody,
      frameInA: btTransform,
      frameInB: btTransform
    );
  }

  class btConstraintSolver {}

  class btDispatcherInfo {
    m_timeStep: number;
    m_stepCount: number;
    m_dispatchFunc: number;
    m_timeOfImpact: number;
    m_useContinuous: boolean;
    m_enableSatConvex: boolean;
    m_enableSPU: boolean;
    m_useEpa: boolean;
    m_allowedCcdPenetration: number;
    m_useConvexConservativeDistanceUtil: boolean;
    m_convexConservativeDistanceThreshold: number;
  }

  class btCollisionWorld {
    getDispatcher(): btDispatcher;
    rayTest(
      rayFromWorld: btVector3,
      rayToWorld: btVector3,
      resultCallback: RayResultCallback
    ): void;
    getPairCache(): btOverlappingPairCache;
    getDispatchInfo(): btDispatcherInfo;
    addCollisionObject(
      collisionObject: btCollisionObject,
      collisionFilterGroup?: number,
      collisionFilterMask?: number
    ): void;
    removeCollisionObject(collisionObject: btCollisionObject): void;
    getBroadphase(): btBroadphaseInterface;
    convexSweepTest(
      castShape: btConvexShape,
      from: btTransform,
      to: btTransform,
      resultCallback: ConvexResultCallback,
      allowedCcdPenetration: number
    ): void;
    contactPairTest(
      colObjA: btCollisionObject,
      colObjB: btCollisionObject,
      resultCallback: ContactResultCallback
    ): void;
    contactTest(
      colObj: btCollisionObject,
      resultCallback: ContactResultCallback
    ): void;
    updateSingleAabb(colObj: btCollisionObject): void;
    setDebugDrawer(debugDrawer: btIDebugDraw): void;
    getDebugDrawer(): btIDebugDraw;
    debugDrawWorld(): void;
    debugDrawObject(
      worldTransform: btTransform,
      shape: btCollisionShape,
      color: btVector3
    ): void;
  }

  class btContactSolverInfo {
    m_splitImpulse: boolean;
    m_splitImpulsePenetrationThreshold: number;
    m_numIterations: number;
  }

  class btDynamicsWorld extends btCollisionWorld {
    addAction(action: btActionInterface): void;
    removeAction(action: btActionInterface): void;
    getSolverInfo(): btContactSolverInfo;
  }

  class btDiscreteDynamicsWorld extends btDynamicsWorld {
    constructor(
      dispatcher: btDispatcher,
      pairCache: btBroadphaseInterface,
      constraintSolver: btConstraintSolver,
      collisionConfiguration: btCollisionConfiguration
    );
    setGravity(gravity: btVector3): void;
    getGravity(): btVector3;
    addRigidBody(body: btRigidBody): void;
    addRigidBody(body: btRigidBody, group: number, mask: number): void;
    removeRigidBody(body: btRigidBody): void;
    addConstraint(
      constraint: btTypedConstraint,
      disableCollisionsBetweenLinkedBodies?: boolean
    ): void;
    removeConstraint(constraint: btTypedConstraint): void;
    stepSimulation(
      timeStep: number,
      maxSubSteps?: number,
      fixedTimeStep?: number
    ): number;
    setContactAddedCallback(funcpointer: number): void;
    setContactProcessedCallback(funcpointer: number): void;
    setContactDestroyedCallback(funcpointer: number): void;
  }

  class btVehicleTuning {
    constructor();
    m_suspensionStiffness: number;
    m_suspensionCompression: number;
    m_suspensionDamping: number;
    m_maxSuspensionTravelCm: number;
    m_frictionSlip: number;
    m_maxSuspensionForce: number;
  }

  class btVehicleRaycasterResult {
    m_hitPointInWorld: btVector3;
    m_hitNormalInWorld: btVector3;
    m_distFraction: number;
  }

  class btVehicleRaycaster {
    castRay(
      from: btVector3,
      to: btVector3,
      result: btVehicleRaycasterResult
    ): void;
  }

  class btDefaultVehicleRaycaster extends btVehicleRaycaster {
    constructor(world: btDynamicsWorld);
  }

  class RaycastInfo {
    m_contactNormalWS: btVector3;
    m_contactPointWS: btVector3;
    m_suspensionLength: number;
    m_hardPointWS: btVector3;
    m_wheelDirectionWS: btVector3;
    m_wheelAxleWS: btVector3;
    m_isInContact: boolean;
    m_groundObject: any;
  }

  class btWheelInfoConstructionInfo {
    m_chassisConnectionCS: btVector3;
    m_wheelDirectionCS: btVector3;
    m_wheelAxleCS: btVector3;
    m_suspensionRestLength: number;
    m_maxSuspensionTravelCm: number;
    m_wheelRadius: number;
    m_suspensionStiffness: number;
    m_wheelsDampingCompression: number;
    m_wheelsDampingRelaxation: number;
    m_frictionSlip: number;
    m_maxSuspensionForce: number;
    m_bIsFrontWheel: boolean;
  }

  class btWheelInfo {
    m_suspensionStiffness: number;
    m_frictionSlip: number;
    m_engineForce: number;
    m_rollInfluence: number;
    m_suspensionRestLength1: number;
    m_wheelsRadius: number;
    m_wheelsDampingCompression: number;
    m_wheelsDampingRelaxation: number;
    m_steering: number;
    m_maxSuspensionForce: number;
    m_maxSuspensionTravelCm: number;
    m_wheelsSuspensionForce: number;
    m_bIsFrontWheel: boolean;
    m_raycastInfo: RaycastInfo;
    m_chassisConnectionPointCS: btVector3;
    constructor(ci: btWheelInfoConstructionInfo);
    getSuspensionRestLength(): number;
    updateWheel(chassis: btRigidBody, raycastInfo: RaycastInfo): void;
    m_worldTransform: btTransform;
    m_wheelDirectionCS: btVector3;
    m_wheelAxleCS: btVector3;
    m_rotation: number;
    m_deltaRotation: number;
    m_brake: number;
    m_clippedInvContactDotSuspension: number;
    m_suspensionRelativeVelocity: number;
    m_skidInfo: number;
  }

  class btActionInterface {
    updateAction(collisionWorld: btCollisionWorld, deltaTimeStep: number): void;
  }

  class btKinematicCharacterController extends btActionInterface {
    constructor(
      ghostObject: btPairCachingGhostObject,
      convexShape: btConvexShape,
      stepHeight: number,
      upAxis?: number
    );
    setUpAxis(axis: number): void;
    setWalkDirection(walkDirection: btVector3): void;
    setVelocityForTimeInterval(velocity: btVector3, timeInterval: number): void;
    warp(origin: btVector3): void;
    preStep(collisionWorld: btCollisionWorld): void;
    playerStep(collisionWorld: btCollisionWorld, dt: number): void;
    setFallSpeed(fallSpeed: number): void;
    setJumpSpeed(jumpSpeed: number): void;
    setMaxJumpHeight(maxJumpHeight: number): void;
    canJump(): boolean;
    jump(): void;
    setGravity(gravity: number): void;
    getGravity(): number;
    setMaxSlope(slopeRadians: number): void;
    getMaxSlope(): number;
    getGhostObject(): btPairCachingGhostObject;
    setUseGhostSweepTest(useGhostObjectSweepTest: boolean): void;
    onGround(): boolean;
    setUpInterpolate(value: boolean): void;
  }

  class btRaycastVehicle extends btActionInterface {
    constructor(
      tuning: btVehicleTuning,
      chassis: btRigidBody,
      raycaster: btVehicleRaycaster
    );
    applyEngineForce(force: number, wheel: number): void;
    setSteeringValue(steering: number, wheel: number): void;
    getWheelTransformWS(wheelIndex: number): btTransform;
    updateWheelTransform(
      wheelIndex: number,
      interpolatedTransform: boolean
    ): void;
    addWheel(
      connectionPointCS0: btVector3,
      wheelDirectionCS0: btVector3,
      wheelAxleCS: btVector3,
      suspensionRestLength: number,
      wheelRadius: number,
      tuning: btVehicleTuning,
      isFrontWheel: boolean
    ): btWheelInfo;
    getNumWheels(): number;
    getRigidBody(): btRigidBody;
    getWheelInfo(index: number): btWheelInfo;
    setBrake(brake: number, wheelIndex: number): void;
    setCoordinateSystem(
      rightIndex: number,
      upIndex: number,
      forwardIndex: number
    ): void;
    getCurrentSpeedKmHour(): number;
    getChassisWorldTransform(): btTransform;
    rayCast(wheel: btWheelInfo): number;
    updateVehicle(step: number): void;
    resetSuspension(): void;
    getSteeringValue(wheel: number): number;
    updateWheelTransformsWS(
      wheel: btWheelInfo,
      interpolatedTransform?: boolean
    ): void;
    setPitchControl(pitch: number): void;
    updateSuspension(deltaTime: number): void;
    updateFriction(timeStep: number): void;
    getRightAxis(): number;
    getUpAxis(): number;
    getForwardAxis(): number;
    getForwardVector(): btVector3;
    getUserConstraintType(): number;
    setUserConstraintType(userConstraintType: number): void;
    setUserConstraintId(uid: number): void;
    getUserConstraintId(): number;
  }

  class btGhostObject extends btCollisionObject {
    constructor();
    getNumOverlappingObjects(): number;
    getOverlappingObject(index: number): btCollisionObject;
  }

  class btPairCachingGhostObject extends btGhostObject {
    constructor();
  }

  class btGhostPairCallback {
    constructor();
  }

  class btSoftBodyWorldInfo {
    constructor();
    air_density: number;
    water_density: number;
    water_offset: number;
    m_maxDisplacement: number;
    water_normal: btVector3;
    m_broadphase: btBroadphaseInterface;
    m_dispatcher: btDispatcher;
    m_gravity: btVector3;
  }

  class Node {
    m_x: btVector3;
    m_q: btVector3;
    m_v: btVector3;
    m_f: btVector3;
    m_n: btVector3;
    m_im: number;
    m_area: number;
  }

  class tNodeArray {
    size(): number;
    at(n: number): Node;
  }

  class Material {
    m_kLST: number;
    m_kAST: number;
    m_kVST: number;
    m_flags: number;
  }

  class tMaterialArray {
    size(): number;
    at(n: number): Material;
  }

  class Anchor {
    m_node: Node;
    m_local: btVector3;
    m_body: btRigidBody;
    m_influence: number;
    m_c0: btMatrix3x3;
    m_c1: btVector3;
    m_c2: number;
  }

  class tAnchorArray {
    size(): number;
    at(n: number): Anchor;
    clear(): void;
    push_back(val: Anchor): void;
    pop_back(): void;
  }

  class Config {
    kVCF: number;
    kDP: number;
    kDG: number;
    kLF: number;
    kPR: number;
    kVC: number;
    kDF: number;
    kMT: number;
    kCHR: number;
    kKHR: number;
    kSHR: number;
    kAHR: number;
    kSRHR_CL: number;
    kSKHR_CL: number;
    kSSHR_CL: number;
    kSR_SPLT_CL: number;
    kSK_SPLT_CL: number;
    kSS_SPLT_CL: number;
    maxvolume: number;
    timescale: number;
    viterations: number;
    piterations: number;
    diterations: number;
    citerations: number;
    collisions: number;
  }

  class btSoftBody extends btCollisionObject {
    constructor(
      worldInfo: btSoftBodyWorldInfo,
      node_count: number,
      x: btVector3,
      m: number[]
    );
    m_cfg: Config;
    m_nodes: tNodeArray;
    m_materials: tMaterialArray;
    m_anchors: tAnchorArray;
    checkLink(node0: number, node1: number): boolean;
    checkFace(node0: number, node1: number, node2: number): boolean;
    appendMaterial(): Material;
    appendNode(x: btVector3, m: number): void;
    appendLink(
      node0: number,
      node1: number,
      mat: Material,
      bcheckexist: boolean
    ): void;
    appendFace(
      node0: number,
      node1: number,
      node2: number,
      mat: Material
    ): void;
    appendTetra(
      node0: number,
      node1: number,
      node2: number,
      node3: number,
      mat: Material
    ): void;
    appendAnchor(
      node: number,
      body: btRigidBody,
      disableCollisionBetweenLinkedBodies: boolean,
      influence: number
    ): void;
    addForce(force: btVector3): void;
    addForce(force: btVector3, node: number): void;
    addAeroForceToNode(windVelocity: btVector3, nodeIndex: number): void;
    getTotalMass(): number;
    setTotalMass(mass: number, fromfaces: boolean): void;
    setMass(node: number, mass: number): void;
    transform(trs: btTransform): void;
    translate(trs: btVector3): void;
    rotate(rot: btQuaternion): void;
    scale(scl: btVector3): void;
    generateClusters(k: number, maxiterations?: number): number;
    generateBendingConstraints(distance: number, mat: Material): number;
    upcast(colObj: btCollisionObject): btSoftBody;
  }

  class btSoftBodyRigidBodyCollisionConfiguration extends btDefaultCollisionConfiguration {
    constructor(info?: btDefaultCollisionConstructionInfo);
  }

  class btSoftBodySolver {}

  class btDefaultSoftBodySolver extends btSoftBodySolver {
    constructor();
  }

  class btSoftBodyArray {
    size(): number;
    at(n: number): btSoftBody;
  }

  class btSoftRigidDynamicsWorld extends btDiscreteDynamicsWorld {
    constructor(
      dispatcher: btDispatcher,
      pairCache: btBroadphaseInterface,
      constraintSolver: btConstraintSolver,
      collisionConfiguration: btCollisionConfiguration,
      softBodySolver: btSoftBodySolver
    );
    addSoftBody(
      body: btSoftBody,
      collisionFilterGroup: number,
      collisionFilterMask: number
    ): void;
    removeSoftBody(body: btSoftBody): void;
    removeCollisionObject(collisionObject: btCollisionObject): void;
    getWorldInfo(): btSoftBodyWorldInfo;
    getSoftBodyArray(): btSoftBodyArray;
  }

  class btSoftBodyHelpers {
    constructor();
    CreateRope(
      worldInfo: btSoftBodyWorldInfo,
      from: btVector3,
      to: btVector3,
      res: number,
      fixeds: number
    ): btSoftBody;
    CreatePatch(
      worldInfo: btSoftBodyWorldInfo,
      corner00: btVector3,
      corner10: btVector3,
      corner01: btVector3,
      corner11: btVector3,
      resx: number,
      resy: number,
      fixeds: number,
      gendiags: boolean
    ): btSoftBody;
    CreatePatchUV(
      worldInfo: btSoftBodyWorldInfo,
      corner00: btVector3,
      corner10: btVector3,
      corner01: btVector3,
      corner11: btVector3,
      resx: number,
      resy: number,
      fixeds: number,
      gendiags: boolean,
      tex_coords: number[]
    ): btSoftBody;
    CreateEllipsoid(
      worldInfo: btSoftBodyWorldInfo,
      center: btVector3,
      radius: btVector3,
      res: number
    ): btSoftBody;
    CreateFromTriMesh(
      worldInfo: btSoftBodyWorldInfo,
      vertices: number[],
      triangles: number[],
      ntriangles: number,
      randomizeConstraints: boolean
    ): btSoftBody;
    CreateFromConvexHull(
      worldInfo: btSoftBodyWorldInfo,
      vertices: btVector3,
      nvertices: number,
      randomizeConstraints: boolean
    ): btSoftBody;
  }

  type Type =
    | btIDebugDraw
    | DebugDrawer
    | btVector3
    | btVector4
    | btQuadWord
    | btQuaternion
    | btMatrix3x3
    | btTransform
    | btMotionState
    | btDefaultMotionState
    | btCollisionObject
    | btCollisionObjectWrapper
    | RayResultCallback
    | ClosestRayResultCallback
    | btConstCollisionObjectArray
    | btScalarArray
    | AllHitsRayResultCallback
    | btManifoldPoint
    | ContactResultCallback
    | ConcreteContactResultCallback
    | LocalShapeInfo
    | LocalConvexResult
    | ConvexResultCallback
    | ClosestConvexResultCallback
    | btCollisionShape
    | btConvexShape
    | btConvexTriangleMeshShape
    | btBoxShape
    | btCapsuleShape
    | btCapsuleShapeX
    | btCapsuleShapeZ
    | btCylinderShape
    | btCylinderShapeX
    | btCylinderShapeZ
    | btSphereShape
    | btMultiSphereShape
    | btConeShape
    | btConeShapeX
    | btConeShapeZ
    | btIntArray
    | btFace
    | btVector3Array
    | btFaceArray
    | btConvexPolyhedron
    | btConvexHullShape
    | btShapeHull
    | btCompoundShape
    | btStridingMeshInterface
    | btIndexedMesh
    | btIndexedMeshArray
    | btTriangleMesh
    | btConcaveShape
    | btEmptyShape
    | btStaticPlaneShape
    | btTriangleMeshShape
    | btBvhTriangleMeshShape
    | btHeightfieldTerrainShape
    | btDefaultCollisionConstructionInfo
    | btDefaultCollisionConfiguration
    | btPersistentManifold
    | btDispatcher
    | btCollisionDispatcher
    | btOverlappingPairCallback
    | btOverlappingPairCache
    | btAxisSweep3
    | btBroadphaseInterface
    | btCollisionConfiguration
    | btDbvtBroadphase
    | btBroadphaseProxy
    | btRigidBodyConstructionInfo
    | btRigidBody
    | btConstraintSetting
    | btTypedConstraint
    | btPoint2PointConstraint
    | btGeneric6DofConstraint
    | btGeneric6DofSpringConstraint
    | btSequentialImpulseConstraintSolver
    | btConeTwistConstraint
    | btHingeConstraint
    | btSliderConstraint
    | btFixedConstraint
    | btConstraintSolver
    | btDispatcherInfo
    | btCollisionWorld
    | btContactSolverInfo
    | btDynamicsWorld
    | btDiscreteDynamicsWorld
    | btVehicleTuning
    | btVehicleRaycasterResult
    | btVehicleRaycaster
    | btDefaultVehicleRaycaster
    | RaycastInfo
    | btWheelInfoConstructionInfo
    | btWheelInfo
    | btActionInterface
    | btKinematicCharacterController
    | btRaycastVehicle
    | btGhostObject
    | btPairCachingGhostObject
    | btGhostPairCallback
    | btSoftBodyWorldInfo
    | Node
    | tNodeArray
    | Material
    | tMaterialArray
    | Anchor
    | tAnchorArray
    | Config
    | btSoftBody
    | btSoftBodyRigidBodyCollisionConfiguration
    | btSoftBodySolver
    | btDefaultSoftBodySolver
    | btSoftBodyArray
    | btSoftRigidDynamicsWorld
    | btSoftBodyHelpers;

  function destroy(obj: Ammo.Type): void;
}
