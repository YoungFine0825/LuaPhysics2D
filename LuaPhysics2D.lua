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
    local x = (0 - value) * vector.y
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
--------------------------------------------------------------
------------------------Matrix2X2 Declare----------------------------
local Matrix2x2 = {}
function Matrix2x2.New(m00,m01,m10,m11)--赋值顺序为行优先
    local ins = {
        m00 = m00 or 0, m01 = m01 or 0,
        m10 = m10 or 0, m11 = m11 or 0
    }
    return ins
end
function Matrix2x2.Radians(radians)
    local cos = Math.cos(radians)
    local sin = Math.sin(radians)
    local ret = Matrix2x2.New(cos, 0 - sin, sin, cos)
    return ret
end
function Matrix2x2.Set(matrix,radians)
    local cos = Math.cos(radians)
    local sin = Math.sin(radians)
    matrix.m00 = cos
    matrix.m01 = 0 - sin
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
    local newMatrix = Matrix2x2.New(
            matrix.m00 * vector.x,matrix.m01 * vector.y,
            matrix.m10 * vector.x,matrix.m11 * vector.y
    )
    return newMatrix
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
            shapeType = PhysicsShapeType.None,
            body = nil,
            radius = 0,
            rotation = Matrix2x2.Radians(0),
            vertexData = PolygonData.New(),
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
        baseTime = 0,
        startTime = 0,
        deltaTime = 1 / 60 / 10 * 1000,
        currentTime = 0,
        frequency = 0,
        accumulator = 0,
        stepsCount = 0,
        gravityForce = Vector2.New(0,9.81),
        bodies = {},
        contacts = {}
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

end

function LuaPhysics2D:UnInit()

end

function LuaPhysics2D:_TriangleBaryCenter(v1,v2,v3)
    local x = (v1.x + v2.x + v3.x ) / 3
    local y = (v1.y + v2.y + v3.y ) / 3
    return Vector2.New(x,y)
end


return LuaPhysics2D
