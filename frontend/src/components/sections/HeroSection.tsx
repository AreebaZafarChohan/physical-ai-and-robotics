import Link from '@docusaurus/Link';
import React from 'react';
import { motion } from 'framer-motion';
import SectionWrapper from '../layout/SectionWrapper';
import Button from '../core/Button';
import MetricBadge from '../core/MetricBadge';
import BookCard from '../core/BookCard';

const HeroSection: React.FC = () => {
  return (
    <SectionWrapper 
      background="gradient" 
      padding="xl" 
      centered={true}
      className="min-h-[80vh] flex items-center"
    >
      <div className="flex flex-col md:flex-row items-center justify-between w-full">
        {/* Left content block */}
        <div className="md:w-1/2 mb-12 md:mb-0">
          <motion.div
            initial={{ opacity: 0, x: -20 }}
            animate={{ opacity: 1, x: 0 }}
            transition={{ duration: 0.5 }}
          >
            <h1 className="text-4xl md:text-6xl font-bold text-white mb-6">
              Physical AI & <span className="text-neon-purple-400">Humanoid Robotics</span>
            </h1>
            <p className="text-xl text-white mb-8 max-w-lg">
              Explore the future of artificial intelligence embodied in physical form. 
              Learn how humanoid robots are reshaping our world through cutting-edge research and applications.
            </p>
            
            {/* Metrics bar */}
            <div className="flex space-x-6 mb-8">
              <MetricBadge value="7" label="Chapters" />
              <MetricBadge value="39" label="Lessons" />
              <MetricBadge value="100+" label="Exercises" />
            </div>
            
            <div className="flex flex-wrap gap-4">
              <Link to="/docs/intro">
                <Button variant="primary" size="lg">
                  Start Learning
                </Button>
              </Link>
              <Link to="/docs/intro">
                <Button variant="outline" size="lg">
                  View Curriculum
                </Button>
              </Link>
            </div>
          </motion.div>
        </div>
        
        {/* Right book illustration block */}
        
      </div>
    </SectionWrapper>
  );
};

export default HeroSection;