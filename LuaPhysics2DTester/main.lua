---
--- Generated by EmmyLua(https://github.com/EmmyLua)
--- Created by yangfan
--- DateTime: 2020/12/6 12:41
--- 基于Love2D引擎的测试程序

local luaPhyClass = require("../LuaPhysics2D")
local luaPhy = luaPhyClass.New()
local worldOrigin = {x = 0,y = 0}
local bodiesId = {}

local function InitLuaPhy()
    love.window.setTitle("Lua Physics 2D Tester -create by yangfan 2020-12-6")
    --
    local wid,hei = love.graphics.getDimensions()
    worldOrigin.x = wid / 2
    worldOrigin.y = hei / 2
    --
    luaPhy:Init()
    local groundBodyId = luaPhy:CreatePhysicsBodyRectangle(350,60,1)
    luaPhy:SetPhysicsBodyEnable(groundBodyId,false)
    luaPhy:SetPhysicsBodyPositon(groundBodyId,nil,50)
    luaPhy:SetPhysicsBodyRotation(groundBodyId,-2)
    --
    local groundBodyId2 = luaPhy:CreatePhysicsBodyRectangle(500,60,1)
    luaPhy:SetPhysicsBodyEnable(groundBodyId2,false)
    luaPhy:SetPhysicsBodyPositon(groundBodyId2,350,-260)
    luaPhy:SetPhysicsBodyRotation(groundBodyId2,4)

    local cubeBodyId = luaPhy:CreatePhysicsBodyRectangle(60,60,50)
    luaPhy:SetPhysicsBodyPositon(cubeBodyId,nil,250)

    --
    local circleBodyId = luaPhy:CreatePhysicsBodyCircle(30,50)
    luaPhy:SetPhysicsBodyPositon(circleBodyId,80,250)
    luaPhy:GetPhysicsBodyWithId(circleBodyId).restitution = 5
    --
    table.insert(bodiesId,groundBodyId)
    table.insert(bodiesId,groundBodyId2)
    table.insert(bodiesId,cubeBodyId)
    table.insert(bodiesId,circleBodyId)
end

local function drawBody(graphics,bodyId)
    local vertexs = luaPhy:GetPhysicsBodyVertexsWithId(bodyId)
    local vertexCnt = #vertexs
    if vertexCnt <= 0 then
        return
    end
    local nextIndex = 0
    local p1 = {x = 0, y = 0}
    local p2 = {x = 0, y = 0}
    for i = 1,vertexCnt do
        p1.x = worldOrigin.x + vertexs[i].x
        p1.y = worldOrigin.y - vertexs[i].y
        nextIndex = i + 1 <= vertexCnt and i + 1 or 1
        p2.x = worldOrigin.x + vertexs[nextIndex].x
        p2.y = worldOrigin.y - vertexs[nextIndex].y
        graphics.line(p1.x,p1.y,p2.x,p2.y)
    end
end

function love.load()
    InitLuaPhy()
end

function love.quit()
    if luaPhy then
        luaPhy:UnInit()
    end
end

function love.keypressed( key, scancode, isrepeat)

end

function love.update(dt)
    if luaPhy then
        luaPhy:RunPhysicsStep(dt * 500)
    end
end

function love.draw()
    love.graphics.setLineWidth(2)
    love.graphics.setLineStyle('smooth')
    for i = 1,#bodiesId do
        love.graphics.setColor(1/i,2/i,3/i,1)
        drawBody(love.graphics,bodiesId[i])
    end
end
