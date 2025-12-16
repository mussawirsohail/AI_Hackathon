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
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Start Learning - 5min ⏱️
          </Link>
        </div>
      </div>
    </header>
  );
}

function LessonHighlight() {
  return (
    <section className={clsx('feature-section', styles.lessonHighlight)}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <Heading as="h2" className={clsx('text--center margin-bottom--lg', styles.featuredLessonsTitle)}>
              Featured Lessons
            </Heading>
          </div>
        </div>
        <div className="row">
          <div className="col col--4 margin-bottom--lg">
            <div className={clsx('feature-card', styles.featureCard)}>
              <img
                src="/1.png"
                alt="ROS 2 Architecture"
                className="lesson-image"
              />
              <Heading as="h3">ROS 2 Framework</Heading>
              <p>Learn the fundamentals of Robot Operating System 2 for building robust robotics applications.</p>
              <Link to="docs/modules/module-1-ros2/lesson-1-introduction-to-ros2" className="button button--primary button--block">
                Start Lesson
              </Link>
            </div>
          </div>
          <div className="col col--4 margin-bottom--lg">
            <div className={clsx('feature-card', styles.featureCard)}>
              <img
                src="/2.png"
                alt="Humanoid Kinematics"
                className="lesson-image"
              />
              <Heading as="h3">Humanoid Kinematics</Heading>
              <p>Understand forward and inverse kinematics for controlling humanoid robot movements.</p>
              <Link to="docs/modules/module-2-digital-twin/lesson-1-gazebo-simulation" className="button button--primary button--block">
                Start Lesson
              </Link>
            </div>
          </div>
          <div className="col col--4 margin-bottom--lg">
            <div className={clsx('feature-card', styles.featureCard)}>
              <img
                src="/3.png"
                alt="Physics Simulation"
                className="lesson-image"
              />
              <Heading as="h3">Physics Simulation</Heading>
              <p>Explore Gazebo and Unity simulation environments for testing Physical AI algorithms.</p>
              <Link to="docs/modules/module-3-ai-brain/lesson-1-introduction-to-isaac" className="button button--primary button--block">
                Start Lesson
              </Link>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

function AdditionalResources() {
  return (
    <section className={clsx('feature-section', styles.resourcesSection)}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <Heading as="h2" className={clsx('text--center margin-bottom--lg', styles.additionalResourcesTitle)}>
              Additional Resources
            </Heading>
          </div>
        </div>
        <div className="row">
          <div className="col col--3">
            <div className={clsx('feature-card', styles.resourceCard)}>
              <Heading as="h3">Tutorials</Heading>
              <p>Step-by-step guides to implement Physical AI concepts.</p>
            </div>
          </div>
          <div className="col col--3">
            <div className={clsx('feature-card', styles.resourceCard)}>
              <Heading as="h3">Case Studies</Heading>
              <p>Real-world applications of Physical AI and humanoid robotics.</p>
            </div>
          </div>
          <div className="col col--3">
            <div className={clsx('feature-card', styles.resourceCard)}>
              <Heading as="h3">Code Examples</Heading>
              <p>Practical code implementations to learn from.</p>
            </div>
          </div>
          <div className="col col--3">
            <div className={clsx('feature-card', styles.resourceCard)}>
              <Heading as="h3">Research Papers</Heading>
              <p>Curated collection of essential Physical AI research.</p>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Your complete guide to Physical AI & Humanoid Robotics">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
        <LessonHighlight />
        <AdditionalResources />
      </main>
    </Layout>
  );
}
