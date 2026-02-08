import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  icon: string;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Physical AI Foundations',
    icon: '🧠',
    description: (
      <>
        Understand the fundamental principles of Physical AI and how embodiment shapes intelligence in robotic systems.
      </>
    ),
  },
  {
    title: 'ROS 2 Integration',
    icon: '🔗',
    description: (
      <>
        Master ROS 2 as the nervous system for humanoid robotics, enabling seamless communication and control.
      </>
    ),
  },
  {
    title: 'Simulation-First Approach',
    icon: '🎮',
    description: (
      <>
        Learn simulation techniques using Gazebo, Unity, and Isaac Sim for safe and efficient development.
      </>
    ),
  },
  {
    title: 'VLA Systems',
    icon: '💬',
    description: (
      <>
        Explore Vision-Language-Action systems for natural human-robot interaction and collaboration.
      </>
    ),
  },
  {
    title: 'Embodied Intelligence',
    icon: '🤖',
    description: (
      <>
        Discover how physical form and environmental interaction drive learning and adaptation in AI systems.
      </>
    ),
  },
  {
    title: 'RAG-Powered Learning',
    icon: '🔍',
    description: (
      <>
        Interactive chatbot that answers questions based solely on textbook content for enhanced learning.
      </>
    ),
  },
];

function Feature({title, icon, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <div className={styles.featureIcon}>
          {icon}
        </div>
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
