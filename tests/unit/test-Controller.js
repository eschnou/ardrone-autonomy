var common   = require('../common');
var assert   = require('assert');
var test     = require('utest');
var sinon    = require('sinon');
var autonomy = require(common.root);

test('Controller', {
    before: function() {
        this.mockClient      = {};
        this.mockClient.on   = sinon.stub();
        this.mockClient.stop = sinon.stub();
    },

    'controller binds on navdata': function() {
        var ctrl = new autonomy.Controller(this.mockClient);
        assert.equal(this.mockClient.on.callCount, 1);
    },

    'disabling the controller stops the drone': function() {
        var ctrl = new autonomy.Controller(this.mockClient);
        ctrl.disable();
        assert.equal(this.mockClient.stop.callCount, 1);
        assert.equal(ctrl._enabled, false);
    },

    'hover assigns current state as goal': function() {
        var ctrl  = new autonomy.Controller(this.mockClient);
        var state = {x: 1, y: 2, z: 3, yaw: 0};

        ctrl._state = state;
        ctrl.hover();
        var goal = ctrl._goal;

        assert.equal(goal.x, state.x);
        assert.equal(goal.y, state.y);
        assert.equal(goal.z, state.z);
        assert.equal(goal.yaw, state.yaw);
    },

    'forward mapping works with different yaw': function() {
        var ctrl  = new autonomy.Controller(this.mockClient);
        ctrl._state = {x: 0, y: 0, z: 1, yaw: 0};

        // Test forward with yaw 0
        ctrl.forward(1);
        assert.equal(ctrl._goal.x, 1);
        assert.equal(ctrl._goal.y, 0);

        // Test forward with yaw 90
        var yaw = 90;
        ctrl._state.yaw = yaw.toRad();
        ctrl.forward(1);
        assert.equal(Math.round(ctrl._goal.x * 1000) / 1000, 0);
        assert.equal(ctrl._goal.y, 1);

        // Test forward with yaw 45
        var yaw = 45;
        ctrl._state.yaw = yaw.toRad();
        ctrl.forward(1);
        assert.equal(Math.round(ctrl._goal.x * 1000) / 1000, Math.round(ctrl._goal.y * 1000) /1000);

        // Test forward with yaw -45
        var yaw = -45;
        ctrl._state.yaw = yaw.toRad();
        ctrl.forward(1);
        assert.equal(Math.round(ctrl._goal.x * 1000) / 1000, -Math.round(ctrl._goal.y * 1000) /1000);
    },

    'right mapping works with different yaw': function() {
        var ctrl  = new autonomy.Controller(this.mockClient);
        ctrl._state = {x: 0, y: 0, z: 1, yaw: 0};

        // Test right with yaw 0
        ctrl.right(1);
        assert.equal(ctrl._goal.x, 0);
        assert.equal(ctrl._goal.y, 1);

        // Test right with yaw 90
        var yaw = 90;
        ctrl._state.yaw = yaw.toRad();
        ctrl.right(1);
        assert.equal(Math.round(ctrl._goal.x * 1000) / 1000, -1);
        assert.equal(Math.round(ctrl._goal.y * 1000) / 1000, 0);

        // Test right with yaw 45
        var yaw = 45;
        ctrl._state.yaw = yaw.toRad();
        ctrl.right(1);
        assert.equal(Math.round(ctrl._goal.x * 1000) / 1000, -Math.round(ctrl._goal.y * 1000) /1000);
    },

    'backward is the inverse of forward': function() {
        var ctrl    = new autonomy.Controller(this.mockClient);
        var cb      = function() {};
        ctrl._state = {x: 0, y: 0, z: 1, yaw: 0};
        sinon.spy(ctrl, 'forward');

        ctrl.backward(1, cb);
        assert(ctrl.forward.calledWith(-1, cb));
    },

    'left is the inverse of right': function() {
        var ctrl    = new autonomy.Controller(this.mockClient);
        var cb      = function() {};
        ctrl._state = {x: 0, y: 0, z: 1, yaw: 0};
        sinon.spy(ctrl, 'right');

        ctrl.left(1, cb);
        assert(ctrl.right.calledWith(-1, cb));
    },

    'zero reset the kalman filter': function() {
        var ctrl    = new autonomy.Controller(this.mockClient);
        sinon.spy(ctrl._ekf, 'reset');

        ctrl.zero();
        assert(ctrl._ekf.reset.calledOnce);
    },

    'up': function() {
        var ctrl  = new autonomy.Controller(this.mockClient);
        var state = {x: 1, y: 2, z: 3, yaw: 0};

        ctrl._state = state;
        ctrl.up(1);
        var goal = ctrl._goal;

        assert.equal(goal.x, state.x);
        assert.equal(goal.y, state.y);
        assert.equal(goal.z, state.z + 1);
        assert.equal(goal.yaw, state.yaw);
    },

    'down is invserse of up': function() {
        var ctrl    = new autonomy.Controller(this.mockClient);
        var cb      = function() {};
        ctrl._state = {x: 0, y: 0, z: 1, yaw: 0};
        sinon.spy(ctrl, 'up');

        ctrl.down(1, cb);
        assert(ctrl.up.calledWith(-1, cb));
    },

    'cannot go too low': function() {
        var ctrl  = new autonomy.Controller(this.mockClient);
        var state = {x: 0, y: 0, z: 1, yaw: 0};

        ctrl._state = state;
        ctrl.down(1);
        var goal = ctrl._goal;

        assert.equal(goal.x, state.x);
        assert.equal(goal.y, state.y);
        assert.equal(goal.z, 0.5);
        assert.equal(goal.yaw, state.yaw);
    },

    'altitude': function() {
        var ctrl  = new autonomy.Controller(this.mockClient);
        var state = {x: 0, y: 0, z: 1, yaw: 0};

        ctrl._state = state;
        ctrl.altitude(3);
        var goal = ctrl._goal;

        assert.equal(goal.x, state.x);
        assert.equal(goal.y, state.y);
        assert.equal(goal.z, 3);
        assert.equal(goal.yaw, state.yaw);
    }
});
