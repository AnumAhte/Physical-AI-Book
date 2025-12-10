import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

type FeatureItem = {
  title: string;
  week: string;
  description: ReactNode;
  link: string;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Introduction to Physical AI',
    week: 'Week 1-2',
    description: (
      <>
        Get started with Physical AI concepts, hardware setup, and the fundamentals
        of embodied intelligence. Learn about the intersection of AI and robotics.
      </>
    ),
    link: '/docs/week-01-02-intro/intro',
  },
  {
    title: 'ROS 2 Fundamentals',
    week: 'Week 3-5',
    description: (
      <>
        Master Robot Operating System 2 (ROS 2) including nodes, topics, services,
        and URDF robot descriptions. Build your first robot applications.
      </>
    ),
    link: '/docs/week-03-05-ros2/ros2-overview',
  },
  {
    title: 'Digital Twin Simulation',
    week: 'Week 6-7',
    description: (
      <>
        Create digital twins using Gazebo and Unity. Learn simulation-based
        development and test your robots in virtual environments.
      </>
    ),
    link: '/docs/week-06-07-digital-twin/gazebo-intro',
  },
  {
    title: 'NVIDIA Isaac Sim',
    week: 'Week 8-10',
    description: (
      <>
        Explore GPU-accelerated simulation with NVIDIA Isaac Sim. Generate synthetic
        data and implement imitation learning for robot training.
      </>
    ),
    link: '/docs/week-08-10-isaac-sim/isaac-intro',
  },
  {
    title: 'Vision-Language-Action Systems',
    week: 'Week 11-13',
    description: (
      <>
        Dive into multimodal AI, transformer policies, and VLA systems. Build
        intelligent robots that can see, understand, and act.
      </>
    ),
    link: '/docs/week-11-13-vla/vla-overview',
  },
  {
    title: 'Capstone Project',
    week: 'Week 13',
    description: (
      <>
        Apply everything you have learned in an end-to-end autonomous humanoid project.
        Implement a complete voice-to-action pipeline.
      </>
    ),
    link: '/docs/week-13-capstone/capstone-overview',
  },
];

function Feature({title, week, description, link}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className={styles.featureCard}>
        <span className={styles.weekBadge}>{week}</span>
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
        <Link className="button button--primary button--sm" to={link}>
          Start Learning
        </Link>
      </div>
    </div>
  );
}

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/week-01-02-intro/intro">
            Start Learning
          </Link>
        </div>
        <div className={styles.heroFeatures}>
          <span>13 Weeks</span>
          <span>|</span>
          <span>ROS 2 + Isaac Sim</span>
          <span>|</span>
          <span>AI-Powered Chatbot</span>
        </div>
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title="Home"
      description="AI-Native Physical AI & Humanoid Robotics Textbook - Learn ROS 2, simulation, and Vision-Language-Action systems">
      <HomepageHeader />
      <main>
        <section className={styles.features}>
          <div className="container">
            <Heading as="h2" className={styles.sectionTitle}>
              Course Modules
            </Heading>
            <div className="row">
              {FeatureList.map((props, idx) => (
                <Feature key={idx} {...props} />
              ))}
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}
