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
    docLink: '/physical-ai-textbook/docs/introduction',
  },
  {
    id: 2,
    title: 'Humanoid Robotics',
    shortTitle: 'Humanoid Robots',
    description: 'Explore hardware design, bipedal locomotion control, and motion planning for humanoid robots like Atlas and Optimus.',
    topics: ['Hardware Design', 'DOF Allocation', 'Bipedal Locomotion', 'Whole-Body Motion Planning'],
    icon: '🦾',
    color: '#4ECDC4',
    docLink: '/physical-ai-textbook/docs/humanoid-robotics',
  },
  {
    id: 3,
    title: 'ROS2 Fundamentals',
    shortTitle: 'ROS2',
    description: 'Master the Robot Operating System 2, including middleware, QoS policies, launch files, and TF2 for robot coordination.',
    topics: ['ROS1 vs ROS2', 'DDS Middleware', 'QoS Policies', 'Launch Files & CLI'],
    icon: '🔧',
    color: '#95E1D3',
    docLink: '/physical-ai-textbook/docs/ros2-fundamentals',
  },
  {
    id: 4,
    title: 'Digital Twin',
    shortTitle: 'Digital Twin',
    description: 'Learn about simulation, physics engines, URDF models, and sim-to-real transfer strategies for robotic systems.',
    topics: ['Physics Engines', 'URDF & Simulation', 'Sensor Simulation', 'Sim-to-Real Transfer'],
    icon: '🌐',
    color: '#F38181',
    docLink: '/physical-ai-textbook/docs/digital-twin',
  },
  {
    id: 5,
    title: 'Vision-Language-Action Systems',
    shortTitle: 'VLA Models',
    description: 'Understand multimodal AI combining vision, language, and action - from RT-1 to PaLM-E and beyond.',
    topics: ['VLA Architecture', 'Multimodal Perception', 'RT-1/RT-2/PaLM-E', 'Training Strategies'],
    icon: '🧠',
    color: '#AA96DA',
    docLink: '/physical-ai-textbook/docs/vision-language-action',
  },
  {
    id: 6,
    title: 'Capstone Project',
    shortTitle: 'CoffeeBot',
    description: 'Build a complete mobile manipulator system (CoffeeBot) integrating all previous concepts with ROS2 and VLA models.',
    topics: ['Mobile Manipulation', 'Navigation & Manipulation', 'End-to-End Integration', 'Deployment'],
    icon: '☕',
    color: '#FCBAD3',
    docLink: '/physical-ai-textbook/docs/capstone',
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
