# LLM Cognitive Planning: From Natural Language to Robot Task Execution

## Learning Objectives

-   Understand how Large Language Models (LLMs) can be used for high-level robot planning.
-   Explore techniques for grounding LLM outputs into executable robot actions.
-   Learn about function calling and tool use with LLMs in robotics contexts.

## Core Concepts

Traditional robot planning often involves complex symbolic AI, state machines, or motion planners that require explicit programming for every possible scenario. Large Language Models (LLMs) offer a revolutionary alternative: by leveraging their vast knowledge and reasoning capabilities, they can translate high-level natural language goals into a sequence of executable robot actions. This is often referred to as "Cognitive Planning" or "LLM-based Task Planning."

**The LLM as a "Robot Brain":**
Instead of directly controlling low-level movements, the LLM acts as a high-level cognitive planner. It receives a natural language command (e.g., "Go to the kitchen and grab a soda") and breaks it down into sub-goals and a sequence of atomic actions that the robot can perform.

**Key Techniques for LLM Cognitive Planning:**

1.  **Prompt Engineering**: Crafting effective prompts to guide the LLM to generate desired action sequences. This includes:
    -   **Few-shot prompting**: Providing examples of natural language commands and their corresponding robot action sequences.
    -   **Chain-of-Thought (CoT) prompting**: Encouraging the LLM to "think step-by-step" before producing the final action.
    -   **Role-playing**: Instructing the LLM to act as a "robot planner" or "task manager."

2.  **Function Calling / Tool Use**: LLMs can be equipped with a set of "tools" or "functions" that they can call to interact with the real world or a robot's API. The LLM's task is not to execute these functions but to decide *when* and *with what arguments* to call them.
    -   **Tool Definition**: Providing the LLM with descriptions of available robot actions (e.g., `move_to_location(x, y)`, `pick_up_object(object_name)`, `detect_object()`).
    -   **LLM Invocation**: The LLM outputs a structured call to one or more of these tools.
    -   **Execution**: A separate "Executor" component takes the LLM's function call, translates it into ROS 2 commands, and executes it on the robot.

3.  **Feedback Loop**: The robot's environment (e.g., sensor readings, success/failure of actions) provides feedback to the LLM. If an action fails or the environment changes, the LLM can re-plan or adjust its strategy.

**ROS 2 Integration:**
Each "tool" or "function" exposed to the LLM can correspond to a ROS 2 Action, Service, or even a Topic publisher. For example:
-   `move_to_location` could map to a `NavigateToPose` ROS 2 Action.
-   `pick_up_object` could map to a custom `PickObject` ROS 2 Action.
-   `get_robot_status` could map to a ROS 2 Service call.

## Step-by-Step Lab: Designing an LLM-Guided Robot Planner (Conceptual)

This lab will conceptually walk through how an LLM can be integrated into a robot's planning pipeline. Full implementation requires access to LLM APIs (e.g., OpenAI, Google Gemini) and a functional ROS 2 robot stack.

### Code Examples (Conceptual)

Imagine a robot that can `move(location)`, `detect(object_name)`, and `grasp(object_name)`.

1.  **Define Robot Capabilities (Tools)**:
    ```python
    # Conceptual: robot_tools.py
    def move(location: str):
        """Moves the robot to a specified location (e.g., 'kitchen', 'bedroom')."""
        print(f"Robot executing: Moving to {location}")
        # In ROS 2, this would send a Navigation goal
        return {"status": "success", "message": f"Reached {location}"}

    def detect_object(object_name: str):
        """Detects if a specified object is visible to the robot.
        Returns a dictionary with 'found' (bool) and 'position' (tuple or None)."""
        print(f"Robot executing: Detecting {object_name}")
        # In ROS 2, this would involve a perception pipeline (e.g., Isaac ROS)
        # Placeholder for simulation/real detection
        if object_name == "red cube":
            return {"status": "success", "found": True, "position": (0.5, 0.2, 0.1)}
        return {"status": "success", "found": False}

    def grasp_object(object_name: str):
        """Attempts to grasp a specified object at its detected position."""
        print(f"Robot executing: Grasping {object_name}")
        # In ROS 2, this would trigger a manipulation action
        return {"status": "success", "message": f"Grasped {object_name}"}

    robot_tools = [move, detect_object, grasp_object]
    ```

2.  **LLM Interaction (Python with Function Calling)**:
    ```python
    # Conceptual: llm_planner.py
    # from openai import OpenAI # or google.generativeai

    def call_llm_for_plan(user_command: str, available_tools: list):
        """Uses an LLM to generate a sequence of tool calls based on a user command."""
        # This is a highly simplified conceptual example
        # In reality, this involves complex API calls and JSON parsing
        if "pick up the red cube" in user_command.lower():
            # LLM would output something like:
            plan = [
                {"function": "move", "args": {"location": "table"}},
                {"function": "detect_object", "args": {"object_name": "red cube"}},
                {"function": "grasp_object", "args": {"object_name": "red cube"}}
            ]
        else:
            plan = [{"function": "move", "args": {"location": "unknown"}}]
        return plan

    def execute_plan(plan: list, tool_functions: dict):
        """Executes a plan generated by the LLM."""
        for step in plan:
            func_name = step["function"]
            args = step["args"]
            if func_name in tool_functions:
                result = tool_functions[func_name](**args)
                print(f"Step '{func_name}' result: {result}")
                if result.get("status") != "success":
                    print(f"Plan failed at step {func_name}. Re-planning needed.")
                    return False
            else:
                print(f"Unknown function in plan: {func_name}")
                return False
        return True

    # Main execution flow
    user_command = "Please pick up the red cube from the table."
    # tool_functions_map = {func.__name__: func for func in robot_tools}
    # llm_generated_plan = call_llm_for_plan(user_command, robot_tools)
    # execute_plan(llm_generated_plan, tool_functions_map)
    ```

### Hardware/Cloud Alternative

-   **Local LLMs**: Running powerful LLMs locally requires significant GPU resources (e.g., NVIDIA A100 or H100). Smaller, quantized models might run on consumer GPUs or even Jetson devices, but with reduced capabilities.
-   **Cloud LLM APIs**: Most advanced LLM cognitive planning is currently done via cloud APIs (e.g., OpenAI GPT series, Google Gemini). This requires an internet connection and API keys.

## Summary

LLM Cognitive Planning allows robots to interpret complex natural language commands and break them into executable action sequences. By exposing robot capabilities as "tools" to an LLM, and then having an "Executor" component translate the LLM's function calls into ROS 2 actions, we can create highly flexible and intelligent robotic systems. This approach significantly simplifies the programming of complex robot behaviors.

## Assessment / Mini Project

1.  **Question**: Explain the role of "Function Calling" in LLM Cognitive Planning for robots.
2.  **Question**: What is an advantage of using an LLM for high-level robot planning compared to traditional state machines?
3.  **Mini-Project**: Consider a robot with capabilities: `open_door(door_id)`, `navigate_to_room(room_name)`, `find_person(person_name)`. Design a prompt (conceptually) for an LLM that would allow it to respond to the command "Find John in the office and open the door for him." Outline the expected LLM output (sequence of function calls).
