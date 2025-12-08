# Chapter 10: Large Agentic Models & Planning

## Learning Objectives

After completing this chapter, you will be able to:

- **Understand** foundation models (LLMs, vision transformers) and prompting strategies for robotics
- **Implement** reasoning systems that decompose tasks into subtasks
- **Design** action spaces and state representations for AI agents
- **Evaluate** agent performance on planning tasks with grounding to physical systems

## Overview

Large language models (LLMs) like GPT-4 and specialized robotics models (RT-1, RT-2) can understand natural language commands and generate executable robot actions. This chapter covers building planning systems where agents reason about tasks, decompose them hierarchically, and generate control sequences.

## Foundation Models for Robotics

### Large Language Models (LLMs)
Models trained on massive text corpora that can:
- **Understand natural language**: Parse commands like "pick up the cup and place it on the table"
- **Reason about tasks**: Break down complex instructions hierarchically
- **Generate code**: Output executable robot control code
- **Few-shot learning**: Learn new tasks from minimal examples

```python
import openai

# Use LLM for task decomposition
response = openai.ChatCompletion.create(
    model="gpt-4",
    messages=[
        {
            "role": "system",
            "content": "You are a robot planning system. Break down natural language commands into executable steps."
        },
        {
            "role": "user",
            "content": "Pick up the red cube and place it on the blue table"
        }
    ]
)

# Output: "1. Locate red cube\n2. Move arm to cube\n3. Grasp cube\n4. Lift cube\n5. Move to blue table\n6. Place cube on table"
print(response.choices[0].message.content)
```

### Vision Transformers
Self-attention based vision models that:
- **Understand scenes**: Identify objects, spatial relationships
- **Process images end-to-end**: No hand-crafted features
- **Generate embeddings**: Semantic understanding of visual content
- **Multi-modal**: Can fuse vision + language representations

## Hierarchical Task Planning

### Task Decomposition
Large tasks require hierarchical breakdown:

```
High-level: "Prepare breakfast"
├── Make coffee
│   ├── Fill kettle with water
│   ├── Turn on kettle
│   ├── Wait for heating
│   ├── Add coffee to cup
│   └── Pour hot water
├── Make toast
│   ├── Place bread in toaster
│   ├── Set timer
│   └── Retrieve toast when done
└── Set table
    ├── Place plate
    ├── Place cup
    └── Add utensils
```

```python
from typing import List

class TaskPlanner:
    def __init__(self, llm_model='gpt-4'):
        self.model = llm_model

    def decompose_task(self, high_level_task: str) -> List[str]:
        """Use LLM to decompose task into subtasks"""
        prompt = f"""
        Break down this task into detailed executable subtasks:
        Task: {high_level_task}

        Output format:
        1. Subtask 1
        2. Subtask 2
        ...
        """

        response = openai.ChatCompletion.create(
            model=self.model,
            messages=[{"role": "user", "content": prompt}],
            temperature=0.3  # Lower temperature for consistency
        )

        # Parse response into list
        subtasks = response.choices[0].message.content.split('\n')
        return [t.strip() for t in subtasks if t.strip()]

# Usage
planner = TaskPlanner()
subtasks = planner.decompose_task("Pick up cup and fill with water")
# Output: ['Locate cup', 'Move arm to cup', 'Grasp cup', 'Move to sink', 'Fill cup with water']
```

## Grounding Language to Actions

### State Representation
Convert observations to language for LLM reasoning:

```python
def observation_to_text(image, depth, joints):
    """Convert sensor data to text description for LLM"""

    # Vision understanding (from CLIP or other VLM)
    scene_description = vision_model.describe_image(image)

    # Spatial relationships
    spatial_info = f"""
    Current scene: {scene_description}
    Robot pose: position={joints['base_position']}, orientation={joints['base_orientation']}
    Objects detected: {detect_objects(image)}
    """

    return spatial_info

# Send to LLM
state_text = observation_to_text(rgb, depth, joint_angles)
response = openai.ChatCompletion.create(
    model="gpt-4-vision",
    messages=[
        {"role": "user", "content": f"Current state: {state_text}. What should I do next?"}
    ]
)
next_action = response.choices[0].message.content
```

### Action Grounding
Map LLM outputs to robot commands:

```python
class ActionExecutor:
    def __init__(self):
        self.action_map = {
            'move_base': self.move_base,
            'move_arm': self.move_arm,
            'grasp': self.grasp,
            'release': self.release,
            'look_at': self.look_at,
        }

    def execute_action(self, action_description: str):
        """Parse LLM action and execute on robot"""

        # Parse action
        action_type, params = self.parse_action(action_description)

        # Execute
        if action_type in self.action_map:
            self.action_map[action_type](**params)

    def parse_action(self, description: str):
        """Convert natural language action to structured command"""
        # Use small LLM or rule-based parser
        if 'move' in description and 'arm' in description:
            target = extract_target_location(description)
            return 'move_arm', {'target': target}
        elif 'grasp' in description or 'pick' in description:
            obj = extract_object(description)
            return 'grasp', {'object': obj}
        # ... more actions
```

## Advanced Planning with Reinforcement Learning

### Policy Training
Train agents to solve tasks with sparse rewards:

```python
import torch
from torch.distributions import Normal

class HierarchicalPolicy(torch.nn.Module):
    def __init__(self, obs_dim, action_dim):
        super().__init__()
        # High-level policy: select subtask
        self.task_encoder = torch.nn.Linear(obs_dim, 64)
        self.task_head = torch.nn.Linear(64, num_subtasks)

        # Low-level policy: execute subtask
        self.action_encoder = torch.nn.Linear(obs_dim + num_subtasks, 128)
        self.mu = torch.nn.Linear(128, action_dim)
        self.log_sigma = torch.nn.Linear(128, action_dim)

    def forward(self, obs):
        # Select subtask
        task_logits = self.task_head(self.task_encoder(obs))
        subtask = torch.argmax(task_logits, dim=-1)

        # Generate action for subtask
        combined = torch.cat([obs, torch.nn.functional.one_hot(subtask, num_subtasks)])
        mu = self.mu(self.action_encoder(combined))
        log_sigma = self.log_sigma(self.action_encoder(combined))

        # Gaussian policy
        dist = Normal(mu, log_sigma.exp())
        action = dist.rsample()

        return action, dist.log_prob(action).sum(dim=-1)
```

## In-Context Learning

### Few-Shot Prompting
Teach agents new tasks with minimal examples:

```python
def few_shot_prompt(task_description: str, examples: List[dict]):
    """Create few-shot prompt for new task"""

    prompt = "Here are examples of similar tasks:\n\n"

    for i, example in enumerate(examples):
        prompt += f"Example {i+1}:\n"
        prompt += f"Input: {example['input']}\n"
        prompt += f"Output: {example['output']}\n\n"

    prompt += f"Now solve this task:\nInput: {task_description}\nOutput:"

    response = openai.ChatCompletion.create(
        model="gpt-4",
        messages=[{"role": "user", "content": prompt}]
    )

    return response.choices[0].message.content

# Usage: Teach robot a new manipulation skill
examples = [
    {'input': 'Pick up sphere and place on shelf', 'output': 'move_arm(target=sphere), grasp(), move_arm(target=shelf), release()'},
    {'input': 'Move box to table', 'output': 'move_arm(target=box), grasp(), move_arm(target=table), release()'}
]
new_skill = few_shot_prompt("Pick up cube and move to bin", examples)
```

## Exercises

**Exercise 10.1**: Implement a task decomposer using an LLM API
**Exercise 10.2**: Create an action executor that maps natural language to joint commands
**Exercise 10.3**: Build a few-shot learning system for teaching new manipulation tasks

## References

1. GPT-4 API Documentation: https://platform.openai.com/docs/
2. RT-1: Robotics Transformer (Brohan et al., 2022): https://arxiv.org/abs/2212.06817
3. RT-2: Vision-Language-Action Models: https://arxiv.org/abs/2307.15818
4. Chain-of-Thought Prompting (Wei et al., 2022): https://arxiv.org/abs/2201.11903
5. In-Context Learning (Brown et al., 2020): https://arxiv.org/abs/2005.14165
