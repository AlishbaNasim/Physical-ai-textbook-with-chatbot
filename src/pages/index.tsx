import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <div className={`${styles.heroText} animate-in delay-1`}>
            <Heading as="h1" className="hero__title">
              {siteConfig.title}
            </Heading>
            <p className="hero__subtitle">{siteConfig.tagline}</p>
            <div className={styles.heroButtons}>
              <Link
                className="button button--secondary button--lg"
                to="/docs/intro">
                Start Reading - 5min ⏱️
              </Link>
              <Link
                className="button button--outline button--primary button--lg"
                to="/docs/module-1-physical-foundations/">
                Explore Modules
              </Link>
            </div>
          </div>
          <div className={`${styles.heroVisual} animate-in delay-2`}>
            <div className={styles.robotAnimation}>
              <div className={clsx(styles.robotPart, styles.robotHead)}></div>
              <div className={clsx(styles.robotPart, styles.robotBody)}></div>
              <div className={clsx(styles.robotPart, styles.robotArm, styles.robotLeftArm)}></div>
              <div className={clsx(styles.robotPart, styles.robotArm, styles.robotRightArm)}></div>
              <div className={clsx(styles.robotPart, styles.robotLeg, styles.robotLeftLeg)}></div>
              <div className={clsx(styles.robotPart, styles.robotLeg, styles.robotRightLeg)}></div>
            </div>
          </div>
        </div>
      </div>
    </header>
  );
}

function BookOverview() {
  return (
    <section className={styles.bookOverview}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <Heading as="h2" className={`${styles.sectionTitle} animate-in delay-1`}>
              About This Book
            </Heading>
            <p className={`${styles.sectionSubtitle} animate-in delay-2`}>
              A comprehensive guide to Physical AI and Humanoid Robotics
            </p>
          </div>
        </div>
        <div className="row">
          <div className="col col--4">
            <div className={`${clsx(styles.card, styles.featureCard)} animate-in delay-3`}>
              <div className={styles.icon}>🤖</div>
              <Heading as="h3" className={styles.cardTitle}>Physical AI Fundamentals</Heading>
              <p className={styles.cardDescription}>
                Understand the relationship between embodiment and intelligence in Physical AI systems
              </p>
            </div>
          </div>
          <div className="col col--4">
            <div className={`${clsx(styles.card, styles.featureCard)} animate-in delay-4`}>
              <div className={styles.icon}>🔗</div>
              <Heading as="h3" className={styles.cardTitle}>ROS 2 Integration</Heading>
              <p className={styles.cardDescription}>
                Master ROS 2 as the nervous system for humanoid robotics applications
              </p>
            </div>
          </div>
          <div className="col col--4">
            <div className={`${clsx(styles.card, styles.featureCard)} animate-in delay-5`}>
              <div className={styles.icon}>🎮</div>
              <Heading as="h3" className={styles.cardTitle}>Simulation-First</Heading>
              <p className={styles.cardDescription}>
                Learn simulation techniques with Gazebo, Unity, and Isaac Sim
              </p>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

function ModulesOverview() {
  const modules = [
    {
      title: "Module 1: Physical AI Foundations",
      description: "Understanding embodied intelligence and the relationship between physical form and cognition",
      icon: "🧠"
    },
    {
      title: "Module 2: ROS 2 - The Nervous System",
      description: "Using ROS 2 for communication, control, and coordination in humanoid robots",
      icon: "🔌"
    },
    {
      title: "Module 3: Simulation-First Development",
      description: "Developing and testing in simulation before real-world deployment",
      icon: "🧪"
    },
    {
      title: "Module 4: VLA & Conversational Robotics",
      description: "Vision-Language-Action systems for natural human-robot interaction",
      icon: "💬"
    }
  ];

  return (
    <section className={styles.modulesOverview}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <Heading as="h2" className={`${styles.sectionTitle} animate-in delay-1`}>
              Learning Modules
            </Heading>
            <p className={`${styles.sectionSubtitle} animate-in delay-2`}>
              Four comprehensive modules covering all aspects of Physical AI
            </p>
          </div>
        </div>
        <div className="row">
          {modules.map((module, index) => (
            <div key={index} className="col col--3">
              <div className={`${clsx(styles.card, styles.moduleCard)} animate-in delay-${index + 3}`}>
                <div className={styles.moduleIcon}>{module.icon}</div>
                <Heading as="h3" className={styles.cardTitle}>{module.title}</Heading>
                <p className={styles.cardDescription}>{module.description}</p>
                <Link
                  className="button button--sm button--primary"
                  to={`/docs/${module.title.includes('Module 1') ? 'module-1-physical-foundations' : module.title.includes('Module 2') ? 'module-2-ros-nervous-system' : module.title.includes('Module 3') ? 'module-3-simulation-first' : 'module-4-vla-and-conversational-robotics'}/`}>
                  Explore Module
                </Link>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function ChatbotSection() {
  return (
    <section className={styles.chatbotSection}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <Heading as="h2" className={`${styles.sectionTitle} animate-in delay-1`}>
              Interactive Learning Assistant
            </Heading>
            <p className={`${styles.sectionSubtitle} animate-in delay-2`}>
              Ask questions about the textbook content and get answers powered by RAG
            </p>
          </div>
        </div>
        <div className="row">
          <div className="col col--8 col--offset-2">
            <div className={`${styles.chatbotPreview} animate-in delay-3`}>
              <div className={styles.chatHeader}>
                <div className={styles.chatIcon}>🤖</div>
                <div className={styles.chatTitle}>Physical AI Assistant</div>
              </div>
              <div className={styles.chatMessages}>
                <div className={styles.botMessage}>
                  Hello! I'm your Physical AI & Humanoid Robotics assistant. Ask me anything about the textbook content!
                </div>
                <div className={styles.userMessage}>
                  What is the role of embodiment in Physical AI?
                </div>
                <div className={styles.botMessage}>
                  In Physical AI, embodiment refers to the idea that intelligence emerges from the interaction between an agent's body and its environment. The physical form shapes cognitive processes and learning.
                </div>
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
      title={`Home - ${siteConfig.title}`}
      description="A comprehensive textbook on Physical AI & Humanoid Robotics with RAG-powered chatbot">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
        <BookOverview />
        <ModulesOverview />
        <ChatbotSection />
      </main>
    </Layout>
  );
}
