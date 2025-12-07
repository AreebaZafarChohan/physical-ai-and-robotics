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
              <Link to="/docs/category/module-ros">
                <Button variant="outline" size="lg">
                  View Curriculum
                </Button>
              </Link>
            </div>
          </motion.div>
        </div>
        
        {/* Right book illustration block */}
        <div className="md:w-2/5 flex justify-center">
          <motion.div
            initial={{ opacity: 0, scale: 0.8 }}
            animate={{ opacity: 1, scale: 1 }}
            transition={{ duration: 0.5, delay: 0.2 }}
            className="relative"
          >
            <div className="relative w-64 h-80 bg-gradient-to-br from-neon-purple-500/10 to-neon-pink-500/10 rounded-2xl border border-neon-purple-500/30 shadow-2xl p-1">
              <div className="w-full h-full bg-gradient-to-br from-dark-card to-surface rounded-xl flex flex-col items-center justify-center p-6 text-center">
                <div className="mb-6">
                  <div className="w-16 h-16 bg-gradient-to-br from-neon-purple-500 to-neon-pink-500 rounded-full flex items-center justify-center mx-auto mb-4">
                    <span className="text-2xl">🤖</span>
                  </div>
                  <h3 className="text-xl font-bold text-white">Physical AI & Robotics</h3>
                  <p className="text-white text-sm">Embodied Intelligence</p>
                </div>
                
                <div className="text-white text-sm mt-auto">
                  <p>"Exploring the intersection of artificial intelligence and physical form"</p>
                </div>
                
                {/* Visual elements */}
                <div className="absolute -top-4 -right-4 w-8 h-8 rounded-full bg-neon-purple-500 animate-ping"></div>
                <div className="absolute -bottom-4 -left-4 w-6 h-6 rounded-full bg-neon-pink-500 animate-pulse"></div>
              </div>
            </div>
            
            {/* Floating elements */}
            <motion.div 
              className="absolute -top-6 -left-6 w-12 h-12 rounded-full bg-neon-purple-500/20 border border-neon-purple-500/30 flex items-center justify-center"
              animate={{ y: [-10, 10, -10] }}
              transition={{ duration: 3, repeat: Infinity, ease: "easeInOut" }}
            >
              <span>🧠</span>
            </motion.div>
            
            <motion.div 
              className="absolute -bottom-6 -right-6 w-10 h-10 rounded-full bg-neon-pink-500/20 border border-neon-pink-500/30 flex items-center justify-center"
              animate={{ y: [10, -10, 10] }}
              transition={{ duration: 4, repeat: Infinity, ease: "easeInOut" }}
            >
              <span>🦾</span>
            </motion.div>
          </motion.div>
        </div>
      </div>
    </SectionWrapper>
  );
};

export default HeroSection;