--[[

--]]
local LP2D_MAX_BODIES               = 64
local LP2D_MAX_MANIFOLDS            = 4096
local LP2D_MAX_VERTICES             = 24
local LP2D_CIRCLE_VERTICES          = 24
local LP2D_COLLISION_ITERATIONS     = 100
local LP2D_PENETRATION_ALLOWANCE    = 0.05
local LP2D_PENETRATION_CORRECTION   = 0.4
local LP2D_PI                       = 3.14159265358979323846
local LP2D_DEG2RAD                  = (LP2D_PI/180.0)
local LP2D_FLT_MAX                  = 3.402823466e+38--浮点数最大长度
local LP2D_EPSILON                  = 0.000001
local LP2D_K                        = 1/3 --3分之一
-----------------------------------------
local Math = math
function Math.Invserse(value)
    if value == 0 or math.abs(value - 0) < LP2D_EPSILON then
        return 0
    end
    return value * -1
end
-----------------Vector2 Declare---------------------------------
local Vector2 = {}
function Vector2.New(x,y)
    local ins = {x = x or 0,y = y or 0}
    return ins
end
function Vector2.Add(vector1,vector2)
   local x = vector1.x + vector2.x
    local y = vector1.y + vector2.y
    return Vector2.New(x,y)
end
function Vector2.Sub(vector1,vector2)
    local x = vector1.x - vector2.x
    local y = vector1.y - vector2.y
    return Vector2.New(x,y)
end
function Vector2.Cross(vector1,vector2)
    local ret = vector1.x * vector2.y - vector1.y * vector2.x
    return ret
end
function Vector2.CrossByValue(value,vector)
    local x = Math.Invserse(value) * vector.y
    local y = value * vector.x
    local ret = Vector2.New(x,y)
    return ret
end
function Vector2.LenSqr(vector)
    local ret = vector.x * vector.x + vector.y * vector.y
    return ret
end
function Vector2.Dot(vector1,vector2)
    local ret = vector1.x * vector2.x + vector1.y * vector2.y
    return ret
end
function Vector2.DistSqr(vector1,vector2)
    local dir = Vector2.Sub(vector1,vector2)
    local ret = Vector2.Dot(dir,dir)
    return ret
end
function Vector2.Normalize(vector)
    local lenth = Math.sqrt(vector.x * vector.x + vector.y * vector.y)
    if lenth <= 0 then lenth = 1 end
    local ilenth = 1 / lenth
    vector.x = vector.x * ilenth
    vector.y = vector.y * ilenth
end
function Vector2.MulValue(vector,value)
    return Vector2.New(vector.x * value,vector.y * value)
end
function Vector2.DivValue(vector,value)
    return Vector2.New(vector.x / value,vector.y / value)
end
function Vector2.Clone(vector)
    return Vector2.New(vector.x,vector.y)
end
function Vector2.Invserse(vector)
    return Vector2.New(Math.Invserse(vector.x),Math.Invserse(vector.y))
end
--------------------------------------------------------------
------------------------Matrix2X2 Declare----------------------------
local Matrix2x2 = {}
function Matrix2x2.New(m00,m01,m10,m11)--赋值顺序为行优先
    local ins = {
        m00 = m00 or 0,
        m01 = m01 or 0,
        m10 = m10 or 0,
        m11 = m11 or 0
    }
    return ins
end
function Matrix2x2.Radians(radians)
    local cos = Math.cos(radians)
    local sin = Math.sin(radians)
    local ret = Matrix2x2.New(cos, Math.Invserse(sin), sin, cos)
    return ret
end
function Matrix2x2.Set(matrix,radians)
    local cos = Math.cos(radians)
    local sin = Math.sin(radians)
    matrix.m00 = cos
    matrix.m01 = Math.Invserse(sin)
    matrix.m10 = sin
    matrix.m11 = cos
end
function Matrix2x2.Transpose(matrix)
    local newMatrix = Matrix2x2.New(
            matrix.m00,matrix.m10,
            matrix.m01,matrix.m11
    )
    return newMatrix
end
function Matrix2x2.MulVector2(matrix,vector)
    local ret = Vector2.New(
            matrix.m00 * vector.x + matrix.m01 * vector.y,
            matrix.m10 * vector.x + matrix.m11 * vector.y
    )
    return ret
end
------------------------------------------------------
local PhysicsShapeType = {None = 0,Circle = 1,Polygon = 2}
local PolygonData = {
    New = function()
        local ins = {
            vertexCount = 0,
            positions = {},
            normals = {},
        }
        return ins
    end
}
local PhysicsShapeData = {
    New = function()
        local ins = {
            type = PhysicsShapeType.None,
            body = nil,
            radius = 0,
            rotation = Matrix2x2.Radians(0),
            --vertexData = PolygonData.New(),
            vertexCount = 0,
            vertexs = {},
            normals = {},
        }
        return ins
    end
}
local PhysicsBodyData = {
    New = function ()
        local ins = {
            id = 0,
            enable = true,
            position = Vector2.New(0,0),
            velocity = Vector2.New(0,0),
            force = Vector2.New(0,0),
            angularVelocity = 0, --角速度
            torque = 0,--角矩
            orient = 0,--旋转弧度
            inertia = 0,--转动惯量
            inverseInertia = 0,--转动惯量映射到0~1范围
            mass = 0,
            inverseMass = 0,
            staticFriction = 0,--静态摩擦力（0~1）
            dynamicFriction = 0,--动态摩擦力（0~1）
            restitution = 0,--恢复系数（0~1）
            useGravity = true,
            grounded = false,
            freezeOrient = false,
            shape = PhysicsShapeData.New()
        }
        return ins
    end
}
local PhysicsManifoldData = {
    New = function()
        local ins = {
            id = 0,
            bodyA = nil,
            bodyB = nil,
            penetration = 0,
            normal = Vector2.New(0,0),
            contacts = {},
            contactCount = 0,
            staticFriction = 0,--静态摩擦力（0~1）
            dynamicFriction = 0,--动态摩擦力（0~1）
            restitution = 0,--恢复系数（0~1）
        }
        return ins
    end
}

local LuaPhysics2D = {}

function LuaPhysics2D.New()
    local members = {
        _baseTime = 0,
        _startTime = 0,
        _deltaTime = 1 / 60 / 10 * 1000,--单位毫秒
        _currentTime = 0,
        _frequency = 0,
        _accumulator = 0,
        _stepsCount = 0,
        _gravityForce = Vector2.New(0,-9.81),
        _bodies = {},
        _manifolds = {}
    }
    setmetatable(members,{
        __index = function(t,k)
            local ret = LuaPhysics2D[k]
            return ret
        end,
        __newindex = function(t,k,v)
            if type(v) ~= 'function' then
                return
            end
        end
    })
    return members
end

function LuaPhysics2D:Init()
    print("LuaPhysics2D : Initialize successful !")
end

function LuaPhysics2D:UnInit()
    print("LuaPhysics2D : Shut down !")
end

--单位毫秒
function LuaPhysics2D:SetDeltaTime(dt)
    self._deltaTime = dt
end

function LuaPhysics2D:GetAllPhysicsBody()
    return self._bodies
end

function LuaPhysics2D:CreatePhysicsBodyCircle(radius,density)
    return self:CreatePhysicsBodyPolygon(radius,LP2D_CIRCLE_VERTICES,density)
end

function LuaPhysics2D:CreatePhysicsBodyRectangle(width,height,density)
    local bodyId = self:_FindAvailableBodyIndex()
    local bodyData = PhysicsBodyData.New()
    bodyData.id = bodyId
    bodyData.enable = true
    bodyData.staticFriction = 0.4
    bodyData.dynamicFriction = 0.2
    bodyData.useGravity = true
    bodyData.grounded = false
    bodyData.freezeOrient = false
    bodyData.shape.body = bodyData
    bodyData.shape.type = PhysicsShapeType.Polygon
    --
    self:_CreateRectanglePolygon(bodyData.position,width,height,bodyData.shape)
    --
    local center,mass,inertia = self:_DoCalcuCentroidAndMomentofInertia(bodyData,density)
    for i = 1,bodyData.shape.vertexCount do
        bodyData.shape.vertexs[i].x = bodyData.shape.vertexs[i].x - center.x
        bodyData.shape.vertexs[i].y = bodyData.shape.vertexs[i].y - center.y
    end
    bodyData.mass = mass
    bodyData.inverseMass = bodyData.mass ~= 0 and 1 / bodyData.mass or 0
    bodyData.inertia = inertia
    bodyData.inverseInertia = bodyData.inertia ~= 0 and 1 / bodyData.inertia or 0
    --
    self._bodies[#self._bodies + 1] = bodyData
    return bodyId
end

function LuaPhysics2D:CreatePhysicsBodyPolygon(radius,sides,density)
    local bodyId = self:_FindAvailableBodyIndex()
    local bodyData = PhysicsBodyData.New()
    bodyData.id = bodyId
    bodyData.enable = true
    bodyData.staticFriction = 0.4
    bodyData.dynamicFriction = 0.2
    bodyData.useGravity = true
    bodyData.grounded = false
    bodyData.freezeOrient = false
    bodyData.shape.body = bodyData
    bodyData.shape.radius = radius
    bodyData.shape.type = sides == LP2D_CIRCLE_VERTICES and PhysicsShapeType.Circle or PhysicsShapeType.Polygon
    --
    self:_CreateRandomPolygon(bodyData.position,radius,sides,bodyData.shape)
    --
    local center,mass,inertia = self:_DoCalcuCentroidAndMomentofInertia(bodyData,density)
    for i = 1,bodyData.shape.vertexCount do
        bodyData.shape.vertexs[i].x = bodyData.shape.vertexs[i].x - center.x
        bodyData.shape.vertexs[i].y = bodyData.shape.vertexs[i].y - center.y
    end
    bodyData.mass = mass
    bodyData.inverseMass = bodyData.mass ~= 0 and 1 / bodyData.mass or 0
    bodyData.inertia = inertia
    bodyData.inverseInertia = bodyData.inertia ~= 0 and 1 / bodyData.inertia or 0
    --
    self._bodies[#self._bodies + 1] = bodyData
    return bodyId
end

function LuaPhysics2D:GetPhysicsBodyWithId(bodyId)
    local ret = nil
    for i = 1,#self._bodies do
        if self._bodies[i].id == bodyId then
            ret = self._bodies[i]
            break
        end
    end
    return ret
end

function LuaPhysics2D:GetPhysicsBodyVertexsWithId(bodyId)
    local bodyData = self:GetPhysicsBodyWithId(bodyId)
    if not bodyData then
        return {}
    end
    local ret = {}
    for i = 1,bodyData.shape.vertexCount do
        ret[#ret + 1] = Vector2.Add(bodyData.position,Matrix2x2.MulVector2(bodyData.shape.rotation,bodyData.shape.vertexs[i]))
    end
    return ret
end

function LuaPhysics2D:SetPhysicsBodyPositon(bodyId,x,y)
    local bodyData = self:GetPhysicsBodyWithId(bodyId)
    if bodyData then
        bodyData.position.x = x or 0
        bodyData.position.y = y or 0
    end
end

function LuaPhysics2D:SetPhysicsBodyRotation(bodyId,degree)
    local bodyData = self:GetPhysicsBodyWithId(bodyId)
    if bodyData then
        local radians = degree * LP2D_DEG2RAD
        bodyData.orient = radians
        Matrix2x2.Set(bodyData.shape.rotation,radians)
    end
end

function LuaPhysics2D:AddForce(bodyId,force)
    local body = self:GetPhysicsBodyWithId(bodyId)
    if not body then
        return false
    end
    local forceDir = Vector2.New(force.x,force.y)
    body.force.x = body.force.x + forceDir.x
    body.force.y = body.force.y + forceDir.y
    return true
end

function LuaPhysics2D:RunPhysicsStep(dt)
    self._accumulator = self._accumulator + dt
    while(self._accumulator >= self._deltaTime) do
        self:_PhysicsStep()
        self._accumulator = self._accumulator - self._deltaTime
    end
end

-----------------------------------私有方法定义------------------------------------------

function LuaPhysics2D:_DoCalcuCentroidAndMomentofInertia(bodyData,density)
    local center = Vector2.New(0,0)
    local area = 0
    local inertia = 0
    local vertexCnt = bodyData.shape.vertexCount
    for i = 1,vertexCnt do
        local p1 = bodyData.shape.vertexs[i]
        local nextIndex = i + 1 <= vertexCnt and i + 1 or 1
        local p2 = bodyData.shape.vertexs[nextIndex]
        local D = Vector2.Cross(p1,p2)
        local triangleArea = D / 2
        area = area + triangleArea
        --center = center + triangleArea * LP2D_K / (p1 + p2)
        local side = p1.x + p2.x
        if side == 0 then side = 1 end
        center.x = center.x + triangleArea * LP2D_K / side
        side = p1.x + p2.x
        if side == 0 then side = 1 end
        center.y = center.y + triangleArea * LP2D_K / side
        local intx2 = p1.x * p1.x + p2.x * p1.x + p2.x * p2.x
        local inty2 = p1.y * p1.y + p2.y * p1.y + p2.y * p2.y
        inertia = inertia + (0.25 * LP2D_K * D) * (intx2 + inty2)
    end
    center.x = center.x * (1 / area)
    center.y = center.y * (1 / area)
    local mass = density * area
    inertia = inertia * density
    return center,mass,inertia
end

function LuaPhysics2D:_FindAvailableBodyIndex()
    local bodyCount = #self._bodies
    local ts = os.time()
    local id = ts + bodyCount
    return id
end

function LuaPhysics2D:_CreateRandomPolygon(worldPos,radius,sides,shapedData)
    if not shapedData then
        return 1
    end
    --计算顶点位置（局部空间下）
    local angle = 0
    local vertexsList = {}
    for i = 1,sides do
        angle = 360 / sides * i
        local vertexPos = Vector2.New(0,0)
        vertexPos.x = Math.cos(angle * LP2D_DEG2RAD) * radius
        vertexPos.y = Math.sin(angle * LP2D_DEG2RAD) * radius
        vertexsList[i] = vertexPos
    end
    --计算各个边的法线
    local nextVertexIndex = 0
    local normalsList = {}
    local sideVec = nil
    for i = 1,sides do
        nextVertexIndex = i + 1 <= sides and i + 1 or 1
        sideVec = Vector2.Sub(vertexsList[nextVertexIndex],vertexsList[i])
        local normal = Vector2.New(sideVec.y,Math.Invserse(sideVec.x ))
        Vector2.Normalize(normal)
        normalsList[i] = normal
    end
    shapedData.vertexCount = sides
    shapedData.vertexs = vertexsList
    shapedData.normals = normalsList
    return 0
end

function LuaPhysics2D:_CreateRectanglePolygon(worldPos,width,height,shapedData)
    if not shapedData then
        return 1
    end
    local vertexCnt = 4
    local vertexsList = {}
    --计算四个顶点位置（世界空间下，逆时针顺序）
    local halfWid = width / 2
    local halfHei = height / 2
    vertexsList[1] = Vector2.New(worldPos.x + halfWid,worldPos.y - halfHei)--右下
    vertexsList[2] = Vector2.New(worldPos.x + halfWid,worldPos.y + halfHei)--右上
    vertexsList[3] = Vector2.New(worldPos.x - halfWid,worldPos.y + halfHei)--左上
    vertexsList[4] = Vector2.New(worldPos.x - halfWid,worldPos.y - halfHei)--左下
    local nextVertexIndex = 0
    local normalsList = {}
    local sideVec = nil
    for i = 1,vertexCnt do
        nextVertexIndex = i + 1 <= vertexCnt and i + 1 or 1
        sideVec = Vector2.Sub(vertexsList[nextVertexIndex],vertexsList[i])
        local normal = Vector2.New(sideVec.y,Math.Invserse(sideVec.x ))
        Vector2.Normalize(normal)
        normalsList[i] = normal
    end
    shapedData.vertexCount = vertexCnt
    shapedData.vertexs = vertexsList
    shapedData.normals = normalsList
    return 0
end

function LuaPhysics2D:_CreatePolygonWithGivenVertex(worldPos,vertexs,shapeData)
    if type(vertexs) ~= 'table' or #vertexs <= 0 then
        return 1
    end
    if not shapeData then
        return 2
    end
    local vertexCnt = #vertexs
    local vertexsList = {}
    for i = 1,vertexs do
        vertexsList[#vertexsList + 1 ] = Vector2.New(vertexs[i].x , vertexs[i].y)
    end
    local nextVertexIndex = 0
    local normalsList = {}
    local sideVec = nil
    for i = 1,vertexCnt do
        nextVertexIndex = i + 1 <= vertexCnt and i + 1 or 1
        sideVec = Vector2.Sub(vertexsList[nextVertexIndex],vertexsList[i])
        local normal = Vector2.New(sideVec.y,Math.Invserse(sideVec.x ))
        Vector2.Normalize(normal)
        normalsList[i] = normal
    end
    shapeData.vertexCount = vertexCnt
    shapeData.vertexs = vertexsList
    shapeData.normals = normalsList
    return 0
end

function LuaPhysics2D:_PhysicsStep()
    self:_DestroyPhysicsManifoldAll()
    --
    local bodyCount = #self._bodies
    for i = 1,bodyCount do
        self._bodies[i].grounded = false
    end
    --
    local bodyA = nil
    local bodyB = nil
    for i = 1,bodyCount do
        bodyA = self._bodies[i]
        if bodyA ~= nil then
            for j = i + 1,bodyCount do
                bodyB = self._bodies[j]
                if bodyB ~= nil then
                    if bodyA.inverseMass ~= 0 or bodyB.inverseMass ~= 0 then
                        local manifoldData = self:_CreatePhysicsManifold(bodyA,bodyB,false)
                        self:_SolvePhysicsManifold(manifoldData)
                        if manifoldData.contactCount > 0 then
                            local newManifold = self:_CreatePhysicsManifold(bodyA,bodyB,true)
                            newManifold.penetration = manifoldData.penetration
                            newManifold.normal = Vector2.Clone(manifoldData.normal)
                            newManifold.contacts = manifoldData.contacts
                            newManifold.contactCount = manifoldData.contactCount
                            newManifold.dynamicFriction = manifoldData.dynamicFriction
                            newManifold.staticFriction = manifoldData.staticFriction
                            newManifold.restitution = manifoldData.restitution
                        end
                    end
                end
            end
        end
    end
    --遍历所有Body整合受到的力
    for i = 1,bodyCount do
        if self._bodies[i] then
            self:_IntegratePhysicsForces(self._bodies[i])
        end
    end
    --
    local manifoldCount = #self._manifolds
    for i = 1,manifoldCount do
        if self._manifolds[i] then
            self:_InitializePhysicsManifolds(self._manifolds[i])
        end
    end
    --
    for i = 1,LP2D_COLLISION_ITERATIONS do
        for j = 1,manifoldCount do
            if self._manifolds[j] then
                self:_IntegratePhysicsImpulses(self._manifolds[j])
            end
        end
    end
    --计算所有Body的速度
    for i = 1,bodyCount do
        if self._bodies[i] then
            self:_IntegratePhysicsVelocity(self._bodies[i])
        end
    end
    --修正Body的位置
    for i = 1,manifoldCount do
        if self._manifolds[i] then
            self:_CorrectPhysicsPosition(self._manifolds[i])
        end
    end
    --清除所有Body受到的力
    for i = 1,bodyCount do
        if self._bodies[i] then
            self._bodies[i].force = Vector2.New(0,0)
            self._bodies[i].torque = 0
        end
    end
end

function LuaPhysics2D:_FindAvailableManifoldIndex()
    local manifoldCount = #self._manifolds
    local ts = os.time()
    local id = ts + manifoldCount
    return id
end

function LuaPhysics2D:_CreatePhysicsManifold(bodyDataA,bodyDataB,save)
    local manifoldId = self:_FindAvailableManifoldIndex()
    local newManifold = PhysicsManifoldData.New()
    newManifold.id = manifoldId
    newManifold.bodyA = bodyDataA
    newManifold.bodyB = bodyDataB
    if save then
        self._manifolds[#self._manifolds + 1] = newManifold
    end
    return newManifold
end

function LuaPhysics2D:_DestroyPhysicsManifold(manifoldId)
    local ret = false
    if type(manifoldId) == 'number' and manifoldId > 0 then
        for i = #self._manifolds,1,-1 do
            if self._manifolds[i].id == manifoldId then
                table.remove(self._manifolds,i)
                ret = true
                break
            end
        end
    end
    return ret
end

function LuaPhysics2D:_DestroyPhysicsManifoldAll()
    self._manifolds = {}
end

function LuaPhysics2D:_SolvePhysicsManifold(manifoldData)
    local shapeTypeA = manifoldData.bodyA.shape.type
    local shapeTypeB = manifoldData.bodyB.shape.type
    if shapeTypeA == PhysicsShapeType.Circle then

        if shapeTypeB == PhysicsShapeType.Circle then
            self:_SolveCircleToCircle(manifoldData)
        elseif shapeTypeB == PhysicsShapeType.Polygon then
            self:_SolveCircleToPolygon(manifoldData)
        end

    elseif shapeTypeA == PhysicsShapeType.Polygon then

        if shapeTypeB == PhysicsShapeType.Circle then
            self:_SolvePolygonToCircle(manifoldData)
        elseif shapeTypeB == PhysicsShapeType.Polygon then
            self:_SolvePolygonToPolygon(manifoldData)
        end

    end
end

function LuaPhysics2D:_SolveCircleToCircle(manifoldData)
    local bodyA = manifoldData.bodyA
    local bodyB = manifoldData.bodyB
    if bodyA == nil or bodyB == nil then
        return
    end
    local normal = Vector2.Sub(bodyB.position,bodyA.position)
    local distSqr = Vector2.LenSqr(normal)
    local radius = bodyA.shape.radius + bodyB.shape.radius

    if distSqr >= radius * radius then
        manifoldData.contactCount = 0
        return
    end

    local distance = Math.sqrt(distSqr)
    manifoldData.contactCount = 1

    if distance == 0 then
        manifoldData.normal = Vector2.New(1,0)
        manifoldData.penetration = bodyA.shape.radius
        manifoldData.contacts[1] = Vector2.Clone(bodyA.position)
    else
        manifoldData.penetration = radius - distance
        local contactNormal = Vector2.DivValue(normal,distance)
        manifoldData.normal = contactNormal
        manifoldData.contacts[1] = Vector2.Add(Vector2.MulValue(contactNormal,bodyA.shape.radius),bodyA.position)
    end

    if not bodyA.grounded then
        bodyA.grounded = manifoldData.normal.y < 0
    end
end

function LuaPhysics2D:_SolveCircleToPolygon(manifoldData)
    local bodyA = manifoldData.bodyA
    local bodyB = manifoldData.bodyB
    if bodyA == nil or bodyB == nil then
        return
    end
    manifoldData.contactCount = 0
    local center = Matrix2x2.MulVector2(Matrix2x2.Transpose(bodyB.shape.rotation),Vector2.Sub(bodyA.position,bodyB.position))
    local separation = -1 * LP2D_FLT_MAX
    local curSeparation = 0
    local faceNormalIndex = 1
    local shapeDataB = bodyB.shape
    for i = 1,shapeDataB.vertexCount do
        curSeparation = Vector2.Dot(shapeDataB.normals[i],Vector2.Sub(center,shapeDataB.vertexs[i]))

        if curSeparation > bodyA.shape.radius then
            return
        end

        if curSeparation > separation then
            separation = curSeparation
            faceNormalIndex = i
        end
    end

    if separation < LP2D_EPSILON then
        local worldNormal = Matrix2x2.MulVector2(shapeDataB.rotation,shapeDataB.normals[faceNormalIndex])
        local contactNormal = Vector2.Invserse(worldNormal)
        manifoldData.normal = contactNormal
        manifoldData.contactCount = 1
        local bodyARadius = bodyA.shape.radius
        manifoldData.contacts[1] = Vector2.Add(Vector2.MulValue(contactNormal,bodyARadius),bodyA.position)
        manifoldData.penetration = bodyARadius
        return
    end

    local v1 = shapeDataB.vertexs[faceNormalIndex]
    local nextIndex = faceNormalIndex + 1 <= shapeDataB.vertexCount and faceNormalIndex + 1 or 1
    local v2 = shapeDataB.vertexs[nextIndex]

    local dot1 = Vector2.Dot( Vector2.Sub(center,v1) , Vector2.Sub(v2,v1) )
    local dot2 = Vector2.Dot( Vector2.Sub(center,v2) , Vector2.Sub(v1,v2) )

    local bodyARadius = bodyA.shape.radius
    manifoldData.penetration = bodyARadius - separation

    if dot1 <= 0 then
        if Vector2.DistSqr(center,v1) > bodyARadius * bodyARadius then
            return
        end
        --BodyA的坐标到BodyB的碰撞边的第一个顶点的方向向量即为碰撞法线
        local normal = Vector2.Sub(v1,center)
        --将法线转换到BodyB的局部空间下
        normal = Matrix2x2.MulVector2(bodyB.shape.rotation,normal)
        Vector2.Normalize(normal)
        manifoldData.normal = normal
        --
        local contactPoint = Matrix2x2.MulVector2(bodyB.shape.rotation,v1)
        contactPoint = Vector2.Add(contactPoint,bodyB.position)
        manifoldData.contacts[1] = contactPoint
        manifoldData.contactCount = 1
    elseif dot2 <= 0 then
        if Vector2.DistSqr(center,v2) > bodyARadius * bodyARadius then
            return
        end
        --
        local normal = Vector2.Sub(v2,center)
        normal = Matrix2x2.MulVector2(bodyB.shape.rotation,normal)
        Vector2.Normalize(normal)
        manifoldData.normal = normal
        --
        local contactPoint = Matrix2x2.MulVector2(bodyB.shape.rotation,v2)
        contactPoint = Vector2.Add(contactPoint,bodyB.position)
        manifoldData.contacts[1] = contactPoint
        manifoldData.contactCount = 1
    else
        local normal = shapeDataB.normals[faceNormalIndex]
        if Vector2.Dot(Vector2.Sub(center,v1),normal) > bodyARadius then
            return
        end
        normal = Matrix2x2.MulVector2(bodyB.shape.rotation,normal)
        local contactNormal = Vector2.Invserse(normal)
        manifoldData.normal = contactNormal
        manifoldData.contacts[1] = Vector2.Add(Vector2.MulValue(contactNormal,bodyARadius),bodyA.position)
        manifoldData.contactCount = 1
    end
end

function LuaPhysics2D:_SolvePolygonToCircle(manifoldData)
    local bodyA = manifoldData.bodyA
    local bodyB = manifoldData.bodyB
    if bodyA == nil or bodyB == nil then return end

    manifoldData.bodyA = bodyB
    manifoldData.bodyB = bodyA
    self:_SolveCircleToPolygon(manifoldData)

    manifoldData.normal.x = Math.Invserse(manifoldData.normal.x)
    manifoldData.normal.y = Math.Invserse(manifoldData.normal.y)
end

function LuaPhysics2D:_SolvePolygonToPolygon(manifoldData)
    if manifoldData.bodyA == nil or manifoldData.bodyB == nil then
        return
    end
    local shapeDataA = manifoldData.bodyA.shape
    local shapeDataB = manifoldData.bodyB.shape
    manifoldData.contactCount = 0

    local faceA,penetrationA = self:_FindAxisLeastPenetration(shapeDataA,shapeDataB)
    if penetrationA >= 0 then return end

    local faceB,penetrationB = self:_FindAxisLeastPenetration(shapeDataB,shapeDataA)
    if penetrationB >= 0 then return end

    local referenceIndex = 0
    local flip = false
    local refShapeData = nil
    local incShapeData = nil
    if self:_BiasGreaterThan(penetrationA,penetrationB) then
        refShapeData = shapeDataA
        incShapeData = shapeDataB
        referenceIndex = faceA
    else
        refShapeData = shapeDataB
        incShapeData = shapeDataA
        referenceIndex = faceB
        flip = true
    end

    local incidentFace = self:_FindIncidentFace(refShapeData,incShapeData,referenceIndex)

    local v1 = Vector2.Clone(refShapeData.vertexs[referenceIndex])
    --将顶点坐标转换到世界空间下
    v1 = Vector2.Add( Matrix2x2.MulVector2(refShapeData.rotation,v1) , refShapeData.body.position)
    referenceIndex = referenceIndex + 1 <= refShapeData.vertexCount and referenceIndex + 1 or 1
    local v2 = Vector2.Clone(refShapeData.vertexs[referenceIndex])
    --将顶点坐标转换到世界空间下
    v2 = Vector2.Add( Matrix2x2.MulVector2(refShapeData.rotation,v2) , refShapeData.body.position)

    local sidePlaneNormal = Vector2.Sub(v2,v1)
    Vector2.Normalize(sidePlaneNormal)

    local negSide = Math.Invserse(Vector2.Dot(sidePlaneNormal,v1))
    local posSide = Vector2.Dot(sidePlaneNormal,v2)

    local invertSideNormal = Vector2.Invserse(sidePlaneNormal)
    local sp = self:_Clip(invertSideNormal,negSide,incidentFace[1],incidentFace[2])
    if sp < 2 then return end
    sp = self:_Clip(sidePlaneNormal,posSide,incidentFace[1],incidentFace[2])
    if sp < 2 then return end

    local refFaceNormal = Vector2.New(sidePlaneNormal.y, 0 - sidePlaneNormal.x)
    local refC = Vector2.Dot(refFaceNormal,v1)
    manifoldData.normal = flip and Vector2.Invserse(refFaceNormal) or refFaceNormal

    local currentPoint = 0
    local separation = Vector2.Dot(refFaceNormal,incidentFace[1]) - refC
    if separation <= 0 then
        currentPoint = currentPoint + 1
        manifoldData.contacts[currentPoint] = Vector2.Clone(incidentFace[1])
        manifoldData.penetration = Math.Invserse(separation)
    else
        manifoldData.penetration = 0
    end

    separation = Vector2.Dot(refFaceNormal,incidentFace[2]) - refC
    if separation <= 0 then
        currentPoint = currentPoint + 1
        manifoldData.contacts[currentPoint] = Vector2.Clone(incidentFace[2])
        manifoldData.penetration = manifoldData.penetration + Math.Invserse(separation)
        manifoldData.penetration = manifoldData.penetration / currentPoint
    end

    manifoldData.contactCount = currentPoint
end

function LuaPhysics2D:_IntegratePhysicsForces(bodyData)
    if bodyData == nil or bodyData.inverseMass == 0 or not bodyData.enable then
        return
    end
    local dtHalf = self._deltaTime / 2
    bodyData.velocity.x = bodyData.velocity.x + (bodyData.force.x * bodyData.inverseMass) * dtHalf
    bodyData.velocity.y = bodyData.velocity.y + (bodyData.force.y * bodyData.inverseMass) * dtHalf
    if bodyData.useGravity then
        local dt = self._deltaTime / 1000 / 2
        bodyData.velocity.x = bodyData.velocity.x + self._gravityForce.x * dt
        bodyData.velocity.y = bodyData.velocity.y + self._gravityForce.y * dt
    end
    if not bodyData.freezeOrient then
        bodyData.angularVelocity = bodyData.angularVelocity + bodyData.torque + bodyData.inverseInertia * dtHalf
    end
end

function LuaPhysics2D:_InitializePhysicsManifolds(manifoldData)
    local bodyA = manifoldData.bodyA
    local bodyB = manifoldData.bodyB

    if bodyA == nil or bodyB == nil then
        return
    end

    manifoldData.restitution = Math.sqrt(bodyA.restitution * bodyB.restitution)
    manifoldData.staticFriction = Math.sqrt(bodyA.staticFriction * bodyB.staticFriction)
    manifoldData.dynamicFriction = Math.sqrt(bodyA.dynamicFriction * bodyB.dynamicFriction)

    local radiusVel = Vector2.New(0,0)
    local gravityForce = Vector2.New(
            self._gravityForce.x * self._deltaTime / 1000,
            self._gravityForce.y * self._deltaTime / 1000
    )
    local gravityForceLenSqr = Vector2.LenSqr(gravityForce) + LP2D_EPSILON

    for i = 1,manifoldData.contactCount do
        local radiusA = Vector2.Sub(manifoldData.contacts[i],bodyA.position)
        local radiusB = Vector2.Sub(manifoldData.contacts[i],bodyB.position)

        local crossA = Vector2.CrossByValue(bodyA.angularVelocity,radiusA)
        local crossB = Vector2.CrossByValue(bodyB.angularVelocity,radiusB)

        radiusVel.x = bodyB.velocity.x + (crossB.x - bodyA.velocity.x - crossA.x)
        radiusVel.y = bodyB.velocity.y + (crossB.y - bodyA.velocity.y - crossA.y)

        if Vector2.LenSqr(radiusVel) < gravityForceLenSqr then
            manifoldData.restitution = 0
        end
    end
end

function LuaPhysics2D:_IntegratePhysicsImpulses(manifoldData)
    local bodyA = manifoldData.bodyA
    local bodyB = manifoldData.bodyB

    if bodyA == nil or bodyB == nil then
        return
    end

    if Math.abs(bodyA.inverseMass + bodyB.inverseMass) <= LP2D_EPSILON then
        bodyA.velocity = Vector2.New(0,0)
        bodyB.velocity = Vector2.New(0,0)
        return
    end

    local radiusVel = Vector2.New(0,0)
    local impluseVel = Vector2.New(0,0)
    local tangent = Vector2.New(0,0)
    local inverseMassSum = 0
    local tangentImpulse = Vector2.New(0,0)
    for i = 1,manifoldData.contactCount do
        local radiusA = Vector2.Sub(manifoldData.contacts[i],bodyA.position)
        local radiusB = Vector2.Sub(manifoldData.contacts[i],bodyB.position)

        local crossA = Vector2.CrossByValue(bodyA.angularVelocity,radiusA)
        local crossB = Vector2.CrossByValue(bodyB.angularVelocity,radiusB)
        --radiusVel = bodyB.velocity + crossB - bodyA.velocity - crossA
        radiusVel.x = bodyB.velocity.x + crossB.x - bodyA.velocity.x - crossA.x
        radiusVel.y = bodyB.velocity.y + crossB.y - bodyA.velocity.y - crossA.y

        local contactVelocity = Vector2.Dot(radiusVel,manifoldData.normal)
        if contactVelocity > 0 then
            return
        end

        local raCrossN = Vector2.Cross(radiusA,manifoldData.normal)
        local rbCrossN = Vector2.Cross(radiusB,manifoldData.normal)

        inverseMassSum = bodyA.inverseMass + bodyB.inverseMass + (raCrossN*raCrossN) * bodyA.inverseInertia + (rbCrossN*rbCrossN) * bodyB.inverseInertia
        --计算推力大小
        local impluse = Math.Invserse(1 + manifoldData.restitution) * contactVelocity
        impluse = impluse / inverseMassSum
        impluse = impluse / manifoldData.contactCount
        --计算推力方向
        impluseVel.x = manifoldData.normal.x * impluse
        impluseVel.y = manifoldData.normal.y * impluse
        --整合A和B的角速度和在推力方向上的速度
        if bodyA.enable then
            local inverseImpluseVel = Vector2.Invserse(impluseVel)
            --velocity = velocity + (impluseVel * -1) * inverseMass
            bodyA.velocity.x = bodyA.velocity.x + inverseImpluseVel.x * bodyA.inverseMass
            bodyA.velocity.y = bodyA.velocity.y + inverseImpluseVel.y * bodyA.inverseMass
            if not bodyA.freezeOrient then
                bodyA.angularVelocity = bodyA.angularVelocity + bodyA.inverseInertia * Vector2.Cross(radiusA,inverseImpluseVel)
            end
        end
        if bodyB.enable then
            --velocity = velocity + impluseVel * inverseMass
            bodyB.velocity.x = bodyB.velocity.x + impluseVel.x * bodyB.inverseMass
            bodyB.velocity.y = bodyB.velocity.y + impluseVel.y * bodyB.inverseMass
            if not bodyB.freezeOrient then
                bodyB.angularVelocity = bodyB.angularVelocity + bodyB.inverseInertia * Vector2.Cross(radiusB,impluseVel)
            end
        end
        --重新计算
        crossA = Vector2.CrossByValue(bodyA.angularVelocity,radiusA)
        crossB = Vector2.CrossByValue(bodyB.angularVelocity,radiusB)
        radiusVel.x = bodyB.velocity.x + crossB.x - bodyA.velocity.x - crossA.x
        radiusVel.y = bodyB.velocity.y + crossB.y - bodyA.velocity.y - crossA.y
        contactVelocity = Vector2.Dot(radiusVel,manifoldData.normal)
        --计算切线
        --tangent = radiusVel - manifoldData.normal * contactVelocity
        tangent.x = radiusVel.x - manifoldData.normal.x * contactVelocity
        tangent.y = radiusVel.y - manifoldData.normal.y * contactVelocity
        --计算切线推力大小
        local impulseTangent = Math.Invserse(Vector2.Dot(radiusVel,tangent))
        impulseTangent = impulseTangent / inverseMassSum
        impulseTangent = impulseTangent / manifoldData.contactCount

        local absImpulseTangent = Math.abs(impulseTangent)
        if absImpulseTangent <= LP2D_EPSILON then
            return
        end
        --计算切线推力方向
        tangentImpulse.x = 0
        tangentImpulse.y = 0
        if absImpulseTangent < impluse * manifoldData.staticFriction then
            --tangentImpulse = tangent * impulseTangent
            tangentImpulse.x = tangent.x * impulseTangent
            tangentImpulse.y = tangent.y * impulseTangent
        else
            --tangentImpulse = tangent * -impluse * manifoldData.dynamicFriction
            tangentImpulse.x = tangent.x * Math.Invserse(impluse) * manifoldData.dynamicFriction
            tangentImpulse.y = tangent.y * Math.Invserse(impluse) * manifoldData.dynamicFriction
        end
        --整合A和B的角速度和在切线方向上的速度
        if bodyA.enable then
            local inverseTanImpluseVel = Vector2.Invserse(tangentImpulse)
            --velocity = velocity + (tangentImpulse * -1) * inverseMass
            bodyA.velocity.x = bodyA.velocity.x + inverseTanImpluseVel.x * bodyA.inverseMass
            bodyA.velocity.y = bodyA.velocity.y + inverseTanImpluseVel.y * bodyA.inverseMass
            if not bodyA.freezeOrient then
                bodyA.angularVelocity = bodyA.angularVelocity + bodyA.inverseInertia * Vector2.Cross(radiusA,inverseTanImpluseVel)
            end
        end
        if bodyB.enable then
            --velocity = velocity + tangentImpulse * inverseMass
            bodyB.velocity.x = bodyB.velocity.x + tangentImpulse.x * bodyB.inverseMass
            bodyB.velocity.y = bodyB.velocity.y + tangentImpulse.y * bodyB.inverseMass
            if not bodyB.freezeOrient then
                bodyB.angularVelocity = bodyB.angularVelocity + bodyB.inverseInertia * Vector2.Cross(radiusB,tangentImpulse)
            end
        end
    end
end

function LuaPhysics2D:_IntegratePhysicsVelocity(bodyData)
    if bodyData == nil or not bodyData.enable then
        return
    end
    local dt = self._deltaTime
    bodyData.position.x = bodyData.position.x + bodyData.velocity.x * dt
    bodyData.position.y = bodyData.position.y + bodyData.velocity.y * dt

    if not bodyData.freezeOrient then
        bodyData.orient = bodyData.orient + bodyData.angularVelocity * dt
    end

    Matrix2x2.Set(bodyData.shape.rotation,bodyData.orient)

    self:_IntegratePhysicsForces(bodyData)
end

function LuaPhysics2D:_CorrectPhysicsPosition(manifoldData)
    local bodyA = manifoldData.bodyA
    local bodyB = manifoldData.bodyB

    if bodyA == nil or bodyB == nil then
        return
    end

    local penetration = Math.max(manifoldData.penetration - LP2D_PENETRATION_ALLOWANCE,0)
    local mass = bodyA.inverseMass + bodyB.inverseMass
    --correctPos = (penetration / mass) * normal * LP2D_PENETRATION_CORRECTION
    local correctPos = Vector2.New(
            (penetration / mass ) * manifoldData.normal.x * LP2D_PENETRATION_CORRECTION,
            (penetration / mass ) * manifoldData.normal.y * LP2D_PENETRATION_CORRECTION
    )

    if bodyA.enable then
        --pos = pos - correctPos * mass
        bodyA.position.x = bodyA.position.x - correctPos.x * bodyA.inverseMass
        bodyA.position.y = bodyA.position.y - correctPos.y * bodyA.inverseMass
    end

    if bodyB.enable then
        --pos = pos + correctPos * mass
        bodyB.position.x = bodyB.position.x + correctPos.x * bodyB.inverseMass
        bodyB.position.y = bodyB.position.y + correctPos.y * bodyB.inverseMass
    end
end

function LuaPhysics2D:_FindAxisLeastPenetration(shapeDataA,shapeDataB)
    local bestDistance = -1 * LP2D_FLT_MAX
    local bestVertexIndex = 1
    local curDistance = 0
    for i = 1,shapeDataA.vertexCount do
        --
        local normalInA = Matrix2x2.MulVector2(shapeDataA.rotation,shapeDataA.normals[i])
        --
        local buT = Matrix2x2.Transpose(shapeDataB.rotation)
        local normalInB = Matrix2x2.MulVector2(buT,normalInA)
        --
        local support = self:_GetSupport(shapeDataB,Vector2.Invserse(normalInB))
        --将A的顶点转换到世界空间下
        local worldVertexA = Matrix2x2.MulVector2(shapeDataA.rotation,shapeDataA.vertexs[i])
        worldVertexA = Vector2.Add(worldVertexA,shapeDataA.body.position)
        --取A的顶点到B的位置的方向向量,并且转换到B的局部空间下。
        local dirB2A = Vector2.Sub(worldVertexA,shapeDataB.body.position)
        dirB2A = Matrix2x2.MulVector2(buT,dirB2A)
        --
        curDistance = Vector2.Dot(normalInB,Vector2.Sub(support,dirB2A))
        if curDistance > bestDistance then
            bestVertexIndex = i
            bestDistance = curDistance
        end
    end
    return bestVertexIndex,bestDistance
end

function LuaPhysics2D:_GetSupport(shapeData,dir)
    local bestDot = -1 * LP2D_FLT_MAX
    local bestVertex = Vector2.New(0,0)
    local dot = 0
    for i = 1,shapeData.vertexCount do
        dot = Vector2.Dot(shapeData.vertexs[i],dir)
        if dot > bestDot then
            bestDot = dot
            bestVertex = Vector2.Clone(shapeData.vertexs[i])
        end
    end
    return bestVertex
end

function LuaPhysics2D:_FindIncidentFace(refShapeData,incShapeData,refNormalIndex)
    local referenceNormal = Vector2.Clone(refShapeData.normals[refNormalIndex])
    --先用reference的旋转矩阵对法线旋转，转换到世界空间下。
    referenceNormal = Matrix2x2.MulVector2(refShapeData.rotation,referenceNormal)
    --再用incident的旋转矩阵，将法线转换到incident的局部空间下
    referenceNormal = Matrix2x2.MulVector2(Matrix2x2.Transpose(incShapeData.rotation),referenceNormal)
    local incVertexIndex = 1
    local minDot = LP2D_FLT_MAX
    local dot = 0
    for i = 1,incShapeData.vertexCount do
        dot = Vector2.Dot(referenceNormal,incShapeData.normals[i])
        if dot < minDot then
            minDot = dot
            incVertexIndex = i
        end
    end
    --将边的两个顶点的坐标转换到世界空间下。
    local v0 = Matrix2x2.MulVector2(incShapeData.rotation,incShapeData.vertexs[incVertexIndex])
    v0 = Vector2.Add(v0,incShapeData.body.position)
    incVertexIndex = incVertexIndex + 1 <= incShapeData.vertexCount and incVertexIndex + 1 or 1
    local v1 = Matrix2x2.MulVector2(incShapeData.rotation,incShapeData.vertexs[incVertexIndex])
    v1 = Vector2.Add(v1,incShapeData.body.position)
    return { v0 , v1 }
end

function LuaPhysics2D:_Clip(normal,clip,faceA,faceB)
    local sp = 0
    local out = {faceA,faceB}
    local distanceA = Vector2.Dot(normal,faceA) - clip
    local distanceB = Vector2.Dot(normal,faceB) - clip

    if distanceA <= 0 then
        sp = sp + 1
        out[sp] = faceA
    end

    if distanceB <= 0 then
        sp = sp + 1
        out[sp] = faceB
    end

    if distanceA * distanceB < 0 then
        sp = sp + 1
        local alpha = distanceA / (distanceA - distanceB)
        local delta = Vector2.Sub(faceA,faceB)
        delta.x = delta.x * alpha
        delta.y = delta.y * alpha
        out[sp] = Vector2.Add(faceA,delta)
    end
    faceA = out[1]
    faceB = out[2]
    return sp
end

function LuaPhysics2D:_BiasGreaterThan(valueA,valueB)
    local ret = valueA >= (valueB * 0.95 + valueA * 0.01)
    return ret
end

function LuaPhysics2D:_TriangleBaryCenter(v1,v2,v3)
    local x = (v1.x + v2.x + v3.x ) / 3
    local y = (v1.y + v2.y + v3.y ) / 3
    return Vector2.New(x,y)
end


return LuaPhysics2D
