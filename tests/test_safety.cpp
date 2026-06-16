#include "test_helpers.hpp"

#include <rozeta/motors.hpp>
#include <rozeta/runtime.hpp>
#include <rozeta/safety.hpp>

using namespace rozeta;

void test_physical_estop_latch_asserts_and_requires_reset()
{
    safety::MockDigitalEmergencyInput input;
    safety::PhysicalEstopLatch latch;

    REQUIRE_TRUE(!latch.latched());

    input.setAsserted(true);
    REQUIRE_TRUE(latch.update(input.read()).ok());
    REQUIRE_TRUE(latch.latched());
    REQUIRE_EQ(latch.reason(), std::string("physical E-STOP asserted"));

    input.setAsserted(false);
    REQUIRE_TRUE(latch.update(input.read()).ok());
    REQUIRE_TRUE(latch.latched());
    REQUIRE_TRUE(!latch.reset(input.read()).ok());

    REQUIRE_TRUE(latch.acknowledgeCleared(input.read()).ok());
    REQUIRE_TRUE(!latch.latched());
}

void test_runtime_physical_estop_latch_forces_fault()
{
    runtime::RuntimeConfig config;
    runtime::MissionRuntime runtime(config);
    runtime::RuntimeInputs inputs;

    inputs.physical_estop_latched = true;
    auto out = runtime.tick(inputs, std::chrono::milliseconds{0});

    REQUIRE_EQ(static_cast<int>(out.phase), static_cast<int>(runtime::MissionPhase::Fault));
    REQUIRE_TRUE(out.request_stop);
    REQUIRE_TRUE(out.emergency_stop);
    REQUIRE_EQ(out.reason, std::string("physical E-STOP latched"));
}

void test_safety_motor_gate_refuses_motion_until_latch_reset()
{
    motors::MockMotorController motors;
    safety::MockDigitalEmergencyInput input;
    safety::PhysicalEstopLatch latch;
    safety::SafetyMotorGate gate(motors, latch);

    REQUIRE_TRUE(gate.setSpeed(0.2, 0.2).ok());

    input.setAsserted(true);
    REQUIRE_TRUE(latch.update(input.read()).ok());
    auto stopped = gate.setSpeed(0.2, 0.2);
    REQUIRE_TRUE(!stopped.ok());
    REQUIRE_EQ(static_cast<int>(stopped.code), static_cast<int>(ErrorCode::EmergencyStopped));
    REQUIRE_TRUE(motors.isEmergencyStopped());

    input.setAsserted(false);
    REQUIRE_TRUE(latch.acknowledgeCleared(input.read()).ok());
    motors.clearEmergencyStop();
    REQUIRE_TRUE(gate.setSpeed(0.2, 0.2).ok());
}
