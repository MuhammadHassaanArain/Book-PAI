import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <span className={styles.badge}>🤖 Advanced Learning Path</span>
          <Heading as="h1" className={styles.heroTitle}>
            {siteConfig.title}
          </Heading>
          <p className={styles.heroSubtitle}>{siteConfig.tagline}</p>
          <p className={styles.heroDescription}>
            Master the convergence of AI and robotics. Build autonomous humanoid systems 
            using ROS 2, NVIDIA Isaac, and Vision-Language-Action models.
          </p>
          <div className={styles.buttons}>
            <Link
              className={clsx('button button--primary button--lg', styles.primaryButton)}
              to="/docs/intro">
              Start Learning 🚀
            </Link>
            <Link
              className={clsx('button button--outline button--lg', styles.secondaryButton)}
              href="https://github.com/MuhammadHassaanArain/Book-PAI.git"
              target="_blank"
              rel="noopener noreferrer"
            >
              View Repo 📚
            </Link>

          </div>
        </div>
      </div>
      <div className={styles.heroWave}></div>
    </header>
  );
}

function CourseModules() {
  const modules = [
    {
      title: 'Module 1: The Robotic Nervous System',
      subtitle: 'ROS 2 Foundation',
      icon: '🧠',
      description: 'Master the middleware that controls robots.',
      topics: [
        'ROS 2 Nodes, Topics & Services',
        'Python Agents with rclpy',
        'URDF for Humanoids',
      ],
      color: '#0ea5e9',
    },
    {
      title: 'Module 2: The Digital Twin',
      subtitle: 'Gazebo & Unity',
      icon: '🌐',
      description: 'Build photorealistic simulation environments.',
      topics: [
        'Physics & Collision Simulation',
        'Human-Robot Interaction',
        'Sensor Simulation (LiDAR, Depth, IMU)',
      ],
      color: '#8b5cf6',
    },
    {
      title: 'Module 3: The AI-Robot Brain',
      subtitle: 'NVIDIA Isaac™',
      icon: '⚡',
      description: 'Advanced perception and autonomous navigation.',
      topics: [
        'Isaac Sim: Synthetic Data Generation',
        'Hardware-Accelerated VSLAM',
        'Nav2 Path Planning',
      ],
      color: '#10b981',
    },
    {
      title: 'Module 4: Vision-Language-Action',
      subtitle: 'VLA Models',
      icon: '🎯',
      description: 'The convergence of LLMs and Robotics.',
      topics: [
        'Voice-to-Action with Whisper',
        'LLM Cognitive Planning',
        'Autonomous Humanoid Capstone',
      ],
      color: '#f59e0b',
    },
  ];

  return (
    <section className={styles.modulesSection}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <Heading as="h2" className={styles.sectionTitle}>
            Learning Path
          </Heading>
          <p className={styles.sectionSubtitle}>
            Four comprehensive modules taking you from fundamentals to autonomous systems
          </p>
        </div>
        <div className={styles.modulesGrid}>
          {modules.map((module, idx) => (
            <div key={idx} className={styles.moduleCard}>
              <div className={styles.moduleIcon} style={{backgroundColor: module.color + '20'}}>
                <span style={{color: module.color}}>{module.icon}</span>
              </div>
              <div className={styles.moduleContent}>
                <span className={styles.moduleNumber}>Module {idx + 1}</span>
                <Heading as="h3" className={styles.moduleTitle}>
                  {module.title}
                </Heading>
                <p className={styles.moduleSubtitle}>{module.subtitle}</p>
                <p className={styles.moduleDescription}>{module.description}</p>
                <ul className={styles.topicsList}>
                  {module.topics.map((topic, topicIdx) => (
                    <li key={topicIdx}>
                      <span className={styles.checkmark}>✓</span>
                      {topic}
                    </li>
                  ))}
                </ul>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function TechStack() {
  const technologies = [
    { name: 'ROS 2', icon: '🤖' },
    { name: 'Gazebo', icon: '🔧' },
    { name: 'Unity', icon: '🎮' },
    { name: 'NVIDIA Isaac', icon: '⚡' },
    { name: 'Python', icon: '🐍' },
    { name: 'OpenAI', icon: '🧠' },
    { name: 'Computer Vision', icon: '👁️' },
    { name: 'LLMs', icon: '💬' },
  ];

  return (
    <section className={styles.techSection}>
      <div className="container">
        <Heading as="h2" className={styles.sectionTitle}>
          Technology Stack
        </Heading>
        <div className={styles.techGrid}>
          {technologies.map((tech, idx) => (
            <div key={idx} className={styles.techBadge}>
              <span className={styles.techIcon}>{tech.icon}</span>
              <span className={styles.techName}>{tech.name}</span>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function CapstonePreview() {
  return (
    <section className={styles.capstoneSection}>
      <div className="container">
        <div className={styles.capstoneContent}>
          <div className={styles.capstoneText}>
            <span className={styles.capstoneBadge}>🎓 Capstone Project</span>
            <Heading as="h2" className={styles.capstoneTitle}>
              Build an Autonomous Humanoid
            </Heading>
            <p className={styles.capstoneDescription}>
              Your final project: Create a simulated humanoid robot that receives voice 
              commands, plans paths, navigates obstacles, identifies objects using computer 
              vision, and executes manipulation tasks—all autonomously.
            </p>
            <div className={styles.capstoneFeatures}>
              <div className={styles.feature}>
                <span className={styles.featureIcon}>🎤</span>
                <span>Voice Commands</span>
              </div>
              <div className={styles.feature}>
                <span className={styles.featureIcon}>🗺️</span>
                <span>Path Planning</span>
              </div>
              <div className={styles.feature}>
                <span className={styles.featureIcon}>👁️</span>
                <span>Object Detection</span>
              </div>
              <div className={styles.feature}>
                <span className={styles.featureIcon}>🦾</span>
                <span>Manipulation</span>
              </div>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title}`}
      description="Master Physical AI and Humanoid Robotics with ROS 2, NVIDIA Isaac, and Vision-Language-Action models">
      <HomepageHeader />
      <main>
        <CourseModules />
        <TechStack />
        <CapstonePreview />
      </main>
    </Layout>
  );
}