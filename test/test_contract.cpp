#include <gtest/gtest.h>

#include <array>
#include <cstddef>

#include "crane_model/testing/mock_model.hpp"

namespace crane_control
{
inline constexpr std::size_t kActuatedJointCount = 6;
using ActuatedVector = std::array<double, kActuatedJointCount>;

struct VelocityControllerInput
{
  ActuatedVector position{};
  ActuatedVector velocity{};
  ActuatedVector desired_velocity{};
  ActuatedVector hydraulic_pressure_delta{};
};

struct VelocityControllerOutput
{
  ActuatedVector command{};
  bool valid{false};
};

class VelocityControllerContract
{
public:
  virtual ~VelocityControllerContract() = default;
  virtual VelocityControllerOutput evaluate(const VelocityControllerInput & input) const = 0;
};

struct PendulumEstimatorInput
{
  std::array<double, 2> imu_position{};
  std::array<double, 2> imu_velocity{};
  bool sensor_health{false};
};

struct PendulumEstimatorOutput
{
  std::array<double, 2> position{};
  std::array<double, 2> velocity{};
  bool valid{false};
};

class PendulumEstimatorContract
{
public:
  virtual ~PendulumEstimatorContract() = default;
  virtual PendulumEstimatorOutput estimate(const PendulumEstimatorInput & input) const = 0;
};
}  // namespace crane_control

namespace
{
class VelocityDouble final : public crane_control::VelocityControllerContract
{
public:
  crane_control::VelocityControllerOutput evaluate(
    const crane_control::VelocityControllerInput & input) const override
  {
    return {input.desired_velocity, true};
  }
};

class EstimatorDouble final : public crane_control::PendulumEstimatorContract
{
public:
  crane_control::PendulumEstimatorOutput estimate(
    const crane_control::PendulumEstimatorInput & input) const override
  {
    return {input.imu_position, input.imu_velocity, input.sensor_health};
  }
};
}  // namespace

TEST(CraneControlContract, VelocityDoubleHasSixActuatedAxes)
{
  crane_control::VelocityControllerInput input;
  input.desired_velocity.fill(1.25);
  const auto output = VelocityDouble().evaluate(input);
  EXPECT_EQ(crane_control::kActuatedJointCount, 6U);
  EXPECT_TRUE(output.valid);
  EXPECT_EQ(output.command[5], 1.25);
}

TEST(CraneControlContract, W04UsesInstalledModelTransmissionContract)
{
  const auto model = crane_model::testing::MockModel::create(crane_model::Tool::Pzs100);
  ASSERT_TRUE(model.ok());
  const auto result = model.value().transmission(
    crane_model::Q::Zero(), crane_model::DQA::Zero(), crane_model::ChamberPressure{});
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.value().joint_to_cylinder.rows(), 6);
}

TEST(CraneControlContract, W07UsesInstalledModelPassiveEquilibriumContract)
{
  const auto model = crane_model::testing::MockModel::create(crane_model::Tool::Pzs100);
  ASSERT_TRUE(model.ok());
  const auto result = model.value().passive_equilibrium(
    crane_model::QA::Zero(), crane_model::Payload{});
  EXPECT_EQ(result.status().code, crane_model::ErrorCode::InvalidPayload);
}

TEST(CraneControlContract, EstimatorPropagatesHealth)
{
  crane_control::PendulumEstimatorInput input;
  input.sensor_health = true;
  input.imu_position = {0.1, -0.2};
  const auto output = EstimatorDouble().estimate(input);
  EXPECT_TRUE(output.valid);
  EXPECT_DOUBLE_EQ(output.position[1], -0.2);
}
