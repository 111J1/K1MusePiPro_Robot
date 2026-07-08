#include <gtest/gtest.h>

#include <string>

// Test helper: determine which action a task_manager should take for an intent.
// This tests the routing logic in isolation (no ROS needed).
namespace
{

enum class TaskAction {
  kNone,    // task_manager ignores (tts_node/reminder handles)
  kMove,    // send ExecuteTask(action="move") to control_manager
  kFind,    // send ExecuteTask(action="find") to control_manager
  kStop,    // cancel active goal
  kRotate,  // send ExecuteTask(action="rotate")
  kLift,    // send ExecuteTask(action="lift")
};

TaskAction ClassifyIntent(const std::string & intent_name,
                          const std::string & action)
{
  // Only "action" intents are handled
  if (intent_name != "action") {
    return TaskAction::kNone;
  }

  if (action == "move") return TaskAction::kMove;
  if (action == "find") return TaskAction::kFind;
  if (action == "stop") return TaskAction::kStop;
  if (action == "rotate") return TaskAction::kRotate;
  if (action == "lift") return TaskAction::kLift;

  // Unknown action command
  return TaskAction::kNone;
}

}  // namespace

TEST(TaskMapping, MoveIntent)
{
  EXPECT_EQ(ClassifyIntent("action", "move"), TaskAction::kMove);
}

TEST(TaskMapping, FindIntent)
{
  EXPECT_EQ(ClassifyIntent("action", "find"), TaskAction::kFind);
}

TEST(TaskMapping, StopIntent)
{
  EXPECT_EQ(ClassifyIntent("action", "stop"), TaskAction::kStop);
}

TEST(TaskMapping, RotateIntent)
{
  EXPECT_EQ(ClassifyIntent("action", "rotate"), TaskAction::kRotate);
}

TEST(TaskMapping, LiftIntent)
{
  EXPECT_EQ(ClassifyIntent("action", "lift"), TaskAction::kLift);
}

TEST(TaskMapping, QueryIgnored)
{
  EXPECT_EQ(ClassifyIntent("query", ""), TaskAction::kNone);
}

TEST(TaskMapping, ReminderIgnored)
{
  EXPECT_EQ(ClassifyIntent("reminder", "create"), TaskAction::kNone);
}

TEST(TaskMapping, SystemIgnored)
{
  EXPECT_EQ(ClassifyIntent("system", ""), TaskAction::kNone);
}

TEST(TaskMapping, UnknownActionIgnored)
{
  EXPECT_EQ(ClassifyIntent("action", "grasp"), TaskAction::kNone);
}

TEST(TaskMapping, EmptyActionIgnored)
{
  EXPECT_EQ(ClassifyIntent("action", ""), TaskAction::kNone);
}
