# Contributing to Physical AI & Humanoid Robotics Textbook

Thank you for contributing to this open educational resource! This guide covers chapter authoring, content quality standards, and the publication workflow.

---

## Getting Started

### 1. Local Setup

```bash
# Clone repository
git clone <repo-url>
cd physical-ai-textbook

# Frontend (Docusaurus)
cd website
npm install
npm run start  # Dev server on http://localhost:3000

# Backend (FastAPI)
cd ../backend
python -m venv venv
source venv/bin/activate  # Windows: venv\Scripts\activate
pip install -r requirements.txt
python -m uvicorn main:app --reload  # Dev server on http://localhost:8000
```

### 2. Branch Naming Convention

```
docs/<chapter-number>-<chapter-title>

Examples:
- docs/01-introduction
- docs/03-ros2-fundamentals
- docs/06-capstone
```

### 3. Commit Message Convention

```
[chapter] <number>: <title> - <brief description>

Examples:
- [chapter] 3: ROS 2 Fundamentals - Add nodes and topics section
- [chapter] 5: Vision-Language-Action - Complete pipeline example
```

---

## Chapter Authoring Template

### File Structure

**Location**: `website/docs/<chapter-number>-<title>/index.mdx`

### MDX Template

```mdx
# Chapter 3: ROS 2 Fundamentals

**Learning Objectives**:
After completing this chapter, you will be able to:
1. **Understand**: Explain the concept of ROS 2 nodes and their role in distributed robotics systems
2. **Apply**: Create a simple ROS 2 node using rospy and publish to a topic
3. **Analyze**: Distinguish between topics (pub-sub) and services (req-rep) communication patterns
4. **Evaluate**: Assess when to use services vs topics in a robotics system design

---

## Introduction

Brief overview of chapter content and why it matters.

---

## Section 1: Nodes

### Concepts
- Explanation of nodes
- Links to prior chapters: [Chapter 1: Introduction](/docs/01-introduction)

### Code Example

```python
#!/usr/bin/env python3
"""
ROS 2 Node Example
Expected Output:
  Publishing: "Hello ROS 2"
  Subscriber received: "Hello ROS 2"
"""
import rclpy
from std_msgs.msg import String

def talker():
    node = rclpy.create_node('talker')
    publisher = node.create_publisher(String, 'topic', 10)
    msg = String()
    msg.data = 'Hello ROS 2'
    publisher.publish(msg)

if __name__ == '__main__':
    talker()
```

### Key Takeaways
- Nodes are independent processes
- Communicate via topics (pub-sub) or services (req-rep)
- Nodes are composable and reusable

---

## Section 2: Topics & Services

[Content with examples and exercises]

---

## Exercises

### Exercise 1: Create a Simple Publisher
**Difficulty**: Beginner

Create a ROS 2 node that publishes the string "Hello World" to a topic named `hello_topic`.

**Solution**:
```python
# See code example above; modify message to "Hello World"
```

### Exercise 2: Distinguish Communication Patterns
**Difficulty**: Intermediate

Write a comparison of when to use topics vs services. Consider latency, QoS, and use cases.

**Solution**:
- **Topics**: Continuous data streams (sensor data, transformations) - use pub-sub
- **Services**: Request-reply (get state, trigger action) - synchronous
- **Actions**: Long-running tasks with feedback - use for goals

---

## References

Cite sources with full URLs and version numbers:
1. [ROS 2 Documentation - Humble Distribution](https://docs.ros.org/en/humble/) (v2.0)
2. [ROS 2 Design Document](https://design.ros2.org/) (2024)
3. Quigley et al., "Programming Robots with ROS" (O'Reilly, 2015)

---

## [Chapter Summary Diagram]

(Include SVG or PNG with alt text)

```
graph LR
    A[Nodes] --> B[Topics]
    A --> C[Services]
    B --> D[Publishers]
    B --> E[Subscribers]
```
```

### Quality Checklist

Before submitting your chapter, verify:

- [ ] **Learning objectives** declared at top (using Bloom's verbs)
- [ ] **All code examples** tested locally and run without errors
- [ ] **Expected output comments** in code blocks match actual output
- [ ] **Links to prior chapters** use relative Docusaurus paths (e.g., `/docs/01-introduction`)
- [ ] **External links** are current and verified (tested in last week)
- [ ] **Exercises** include solutions (at least 2 exercises per chapter, 1+ per section)
- [ ] **References** cite 5+ credible sources with URLs and versions
- [ ] **Alt text** included for all images/diagrams
- [ ] **Content aligns** with declared learning objectives (spot check per section)
- [ ] **No typos** (spellcheck or manual review)
- [ ] **Markdown syntax** valid (headers, lists, code blocks)

---

## Pull Request Process

### 1. Create Branch & Push Changes

```bash
git checkout -b docs/03-ros2-fundamentals
# Write chapter in website/docs/03-ros2-fundamentals/index.mdx
git add website/docs/03-ros2-fundamentals/
git commit -m "[chapter] 3: ROS 2 Fundamentals - Add nodes and topics section"
git push origin docs/03-ros2-fundamentals
```

### 2. Open PR on GitHub

**Title**: `[Chapter 3] ROS 2 Fundamentals`

**Description**:
```markdown
## Summary
- Adds Chapter 3: ROS 2 Fundamentals
- Covers: Nodes, topics, services, launch files
- ~2,500 words, 3 code examples, 3 exercises

## Learning Objectives Coverage
- [ ] Understand ROS 2 node architecture
- [ ] Apply pub-sub communication patterns
- [ ] Analyze topic vs service use cases
- [ ] Evaluate system design choices

## Testing
- [ ] Code examples run without errors
- [ ] External links verified
- [ ] Learning objectives aligned with content
- [ ] All exercises have solutions

## References
[List 5+ sources verified]
```

### 3. CI Validation (Automatic)

The following checks run automatically:

- ✅ **Markdown Syntax**: No formatting errors
- ✅ **Code Examples**: Run locally; verify expected outputs
- ✅ **Learning Objectives**: Present in chapter header; use Bloom's verbs
- ✅ **External Links**: All verified; no broken links
- ✅ **Docusaurus Build**: `npm run build` succeeds (<4 min)
- ✅ **RAG Indexing**: Chapter chunks index successfully; validation queries pass (≥3 per chapter)

**If any check fails**: CI will comment on the PR with details. Fix issues and push updates.

### 4. Review & Merge

- **Content review** by subject-matter expert (robotics domain)
- **RAG/structure review** by platform maintainer
- Once approved: **Merge to main** (auto-triggers deployment)

### 5. Auto-Deployment (Post-Merge)

- **Docusaurus**: Builds and deploys to GitHub Pages (5 min)
- **RAG indexing**: Chapter ingested into vector DB; validation runs (2-3 min)
- **Status**: Chapter is live and searchable

---

## Content Standards

### Bloom's Taxonomy for Learning Objectives

Use these verbs to match cognitive levels:

| Level | Verbs | Example |
|-------|-------|---------|
| **Remember** | list, name, recall, identify | "List three types of actuators" |
| **Understand** | explain, describe, interpret, classify | "Explain how ROS 2 nodes communicate" |
| **Apply** | use, demonstrate, solve, modify | "Create a ROS 2 node that publishes sensor data" |
| **Analyze** | distinguish, differentiate, compare | "Compare topics vs services communication" |
| **Synthesize** | combine, design, compile, propose | "Design a multi-node system for humanoid control" |
| **Evaluate** | assess, critique, judge | "Evaluate whether Gazebo or Isaac Sim is better for your use case" |

### Code Example Standards

- Every code example MUST be tested locally before commit
- Include **expected output** as a comment in the code block
- Provide **prerequisites** (e.g., "Requires ROS 2 Humble installed")
- Keep examples **short & focused** (~20-30 lines); save large projects for exercises

### External Link Standards

- Use **Docusaurus relative paths** for internal links: `/docs/01-introduction`
- Use **full URLs with version** for external links: `https://docs.ros.org/en/humble/`
- **Verify links weekly** (CI automation checks before merge)
- Include **access date** for citations without stable versioning

### Reference Standards

**Minimum 5 references per chapter** from credible sources:

✅ **Credible**:
- Official documentation (ROS 2, Gazebo, Isaac Sim)
- Academic papers (arxiv.org, IEEE, ACM)
- Technical books (O'Reilly, Pragmatic Programmers)
- Robotics Stack Exchange (for best practices)

❌ **Not credible**:
- Wikipedia (unless cited elsewhere)
- Blog posts without peer review
- YouTube tutorials (unless from official channels)
- Outdated docs (>3 years old)

---

## FAQ

**Q: How long should a chapter be?**
A: 8-12 pages (approximately 2,500-3,500 words). Aim for depth over breadth.

**Q: Can I use images from other sources?**
A: Yes, but include proper attribution in alt text and references. Prefer Creative Commons licensed images or create your own.

**Q: What if my code example has runtime dependencies (ROS 2, Gazebo)?**
A: Include clear prerequisites and installation links. If it requires hardware, mention that explicitly and provide simulation alternatives.

**Q: How do I test my chapter locally?**
A: Run `npm run start` in `website/` folder; navigate to your chapter; verify links work and code examples are readable.

**Q: Can I write in a different language (e.g., Urdu)?**
A: Urdu translation is a post-MVP feature (P3). For now, write in English; translations will be added via Docusaurus i18n workflow.

---

## Support & Questions

- **Technical issues**: Open GitHub issue with `[question]` label
- **Content feedback**: Comment on the PR before merging
- **Architecture questions**: Check `/docs/architecture/decisions/` for ADRs (Architectural Decision Records)

---

**Thank you for contributing!** 🚀
