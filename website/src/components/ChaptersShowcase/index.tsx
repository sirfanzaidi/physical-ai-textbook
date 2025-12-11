import type {ReactNode} from 'react';
import Link from '@docusaurus/Link';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type ChapterCard = {
  id: number;
  title: string;
  shortTitle: string;
  description: string;
  topics: string[];
  icon: string;
  color: string;
  docLink: string;
};

const ChaptersList: ChapterCard[] = [
  {
    id: 1,
    title: 'Introduction to Physical AI',
    shortTitle: 'Physical AI',
    description: 'Understand the fundamentals of embodied AI, perception-action loops, and how physical AI differs from traditional AI.',
    topics: ['Embodied AI', 'Perception-Action Loops', 'Physical vs Traditional AI', 'Applications & Case Studies'],
    icon: '🤖',
    color: '#FF6B6B',
    docLink: '/docs/introduction',
  },
  {
    id: 2,
    title: 'Humanoid Robotics',
    shortTitle: 'Humanoid Robots',
    description: 'Explore hardware design, bipedal locomotion control, and motion planning for humanoid robots like Atlas and Optimus.',
    topics: ['Hardware Design', 'DOF Allocation', 'Bipedal Locomotion', 'Whole-Body Motion Planning'],
    icon: '🦾',
    color: '#4ECDC4',
    docLink: '/docs/humanoid-robotics',
  },
  {
    id: 3,
    title: 'ROS2 Fundamentals',
    shortTitle: 'ROS2',
    description: 'Master the Robot Operating System 2, including middleware, QoS policies, launch files, and TF2 for robot coordination.',
    topics: ['ROS1 vs ROS2', 'DDS Middleware', 'QoS Policies', 'Launch Files & CLI'],
    icon: '🔧',
    color: '#95E1D3',
    docLink: '/docs/ros2-fundamentals',
  },
  {
    id: 4,
    title: 'Digital Twin',
    shortTitle: 'Digital Twin',
    description: 'Learn about simulation, physics engines, URDF models, and sim-to-real transfer strategies for robotic systems.',
    topics: ['Physics Engines', 'URDF & Simulation', 'Sensor Simulation', 'Sim-to-Real Transfer'],
    icon: '🌐',
    color: '#F38181',
    docLink: '/docs/digital-twin',
  },
  {
    id: 5,
    title: 'Vision-Language-Action Systems',
    shortTitle: 'VLA Models',
    description: 'Understand multimodal AI combining vision, language, and action - from RT-1 to PaLM-E and beyond.',
    topics: ['VLA Architecture', 'Multimodal Perception', 'RT-1/RT-2/PaLM-E', 'Training Strategies'],
    icon: '🧠',
    color: '#AA96DA',
    docLink: '/docs/vision-language-action',
  },
  {
    id: 6,
    title: 'Capstone Project',
    shortTitle: 'CoffeeBot',
    description: 'Build a complete mobile manipulator system (CoffeeBot) integrating all previous concepts with ROS2 and VLA models.',
    topics: ['Mobile Manipulation', 'Navigation & Manipulation', 'End-to-End Integration', 'Deployment'],
    icon: '☕',
    color: '#FCBAD3',
    docLink: '/docs/capstone',
  },
  {
    id: 7,
    title: 'Gazebo & Ignition Simulation',
    shortTitle: 'Gazebo/Ignition',
    description: 'Build high-fidelity simulation environments for humanoid robots using Gazebo physics engine and sensor simulation.',
    topics: ['Physics Engines', 'Sensor Simulation', 'Multi-Robot Worlds', 'Ignition Architecture'],
    icon: '⚙️',
    color: '#A8E6CF',
    docLink: '/docs/gazebo-ignition',
  },
  {
    id: 8,
    title: 'Humanoid Robot Modeling',
    shortTitle: 'URDF Modeling',
    description: 'Create precise URDF models representing humanoid robots with accurate kinematics, mass properties, and actuator dynamics.',
    topics: ['URDF Format', 'Forward Kinematics', 'Inverse Kinematics', 'Model Validation'],
    icon: '📐',
    color: '#FFD3B6',
    docLink: '/docs/humanoid-robot-modeling',
  },
  {
    id: 9,
    title: 'NVIDIA Isaac Sim',
    shortTitle: 'Isaac Sim',
    description: 'Leverage GPU-accelerated simulation with NVIDIA Isaac for large-scale training and synthetic data generation.',
    topics: ['USD Format', 'PhysX Physics', 'Domain Randomization', 'Synthetic Data'],
    icon: '🎮',
    color: '#FFAAA5',
    docLink: '/docs/nvidia-isaac-sim',
  },
  {
    id: 10,
    title: 'Large Agentic Models & Planning',
    shortTitle: 'LLM Planning',
    description: 'Use foundation models (LLMs) for task decomposition, reasoning, and grounding language to robot actions.',
    topics: ['Foundation Models', 'Task Planning', 'Action Grounding', 'Few-Shot Learning'],
    icon: '🧬',
    color: '#FF8B94',
    docLink: '/docs/large-agentic-models-planning',
  },
  {
    id: 11,
    title: 'Robotics Control Systems',
    shortTitle: 'Control Theory',
    description: 'Design feedback control systems (PID, impedance, adaptive) for stable and responsive robot behavior.',
    topics: ['PID Control', 'Impedance Control', 'Stability Analysis', 'Multi-Robot Coordination'],
    icon: '🎯',
    color: '#FFB4A2',
    docLink: '/docs/robotics-control-systems',
  },
  {
    id: 12,
    title: 'Sensor Fusion & Perception',
    shortTitle: 'Perception',
    description: 'Fuse multiple sensors (camera, LiDAR, IMU) using Kalman filters, SLAM, and deep learning for robust perception.',
    topics: ['Kalman Filtering', 'Visual Odometry', 'SLAM', 'Object Detection'],
    icon: '👁️',
    color: '#E5989B',
    docLink: '/docs/sensor-fusion-perception',
  },
  {
    id: 13,
    title: 'Practical Humanoid Use Cases',
    shortTitle: 'Applications',
    description: 'Explore real-world deployments in manufacturing, logistics, healthcare, and disaster response.',
    topics: ['Manufacturing', 'Logistics', 'Healthcare', 'ROI Analysis'],
    icon: '🏭',
    color: '#D5B5B0',
    docLink: '/docs/practical-humanoid-use-cases',
  },
  {
    id: 14,
    title: 'RAG & AI Chatbot for Robotics',
    shortTitle: 'RAG Chatbot',
    description: 'Build intelligent chatbots using Retrieval-Augmented Generation to answer robotics questions with citations.',
    topics: ['Vector Databases', 'Semantic Search', 'LLM Augmentation', 'Chatbot Deployment'],
    icon: '💬',
    color: '#C9ADA7',
    docLink: '/docs/rag-ai-chatbot-robotics',
  },
  {
    id: 15,
    title: 'Robotics Cloud Deployment',
    shortTitle: 'Cloud Deployment',
    description: 'Deploy robot stacks in the cloud with containerization, orchestration, and fleet management at scale.',
    topics: ['Containerization', 'Kubernetes', 'Fleet Management', 'Distributed Training'],
    icon: '☁️',
    color: '#9A8C98',
    docLink: '/docs/robotics-cloud-deployment',
  },
];

interface ChapterItemProps extends ChapterCard {}

function ChapterItem({
  title,
  description,
  topics,
  icon,
  color,
  docLink,
}: ChapterItemProps) {
  return (
    <Link to={docLink} className={styles.cardLink}>
      <div className={styles.card} style={{borderTopColor: color}}>
        <div className={styles.cardIcon}>{icon}</div>
        <div className={styles.cardContent}>
          <Heading as="h3" className={styles.cardTitle}>
            {title}
          </Heading>
          <p className={styles.cardDescription}>{description}</p>
          <div className={styles.topics}>
            {topics.map((topic, idx) => (
              <span key={idx} className={styles.topic} style={{backgroundColor: color + '20', borderColor: color}}>
                {topic}
              </span>
            ))}
          </div>
        </div>
        <div className={styles.cardFooter}>
          <span className={styles.readMore}>Read Chapter →</span>
        </div>
      </div>
    </Link>
  );
}

export default function ChaptersShowcase(): ReactNode {
  return (
    <section className={styles.showcase}>
      <div className="container">
        <div className={styles.header}>
          <Heading as="h2" className={styles.title}>
            📚 Learn Physical AI & Humanoid Robotics
          </Heading>
          <p className={styles.subtitle}>
            Comprehensive chapters covering embodied AI, robotics control, simulation, and advanced AI systems
          </p>
        </div>
        <div className={styles.grid}>
          {ChaptersList.map((chapter) => (
            <ChapterItem key={chapter.id} {...chapter} />
          ))}
        </div>
      </div>
    </section>
  );
}
