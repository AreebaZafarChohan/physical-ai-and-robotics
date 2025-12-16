import React from 'react';
import { motion } from 'framer-motion';
import Button from '../core/Button';

const CTAFooterSection: React.FC = () => {
  return (
    <div className="relative overflow-hidden">
      {/* Animated background elements */}
      <div className="absolute inset-0 bg-gradient-to-r from-neon-purple-500/10 to-neon-pink-500/10">
        <div className="absolute top-1/4 left-1/4 w-64 h-64 bg-neon-purple-500/5 rounded-full filter blur-3xl animate-pulse"></div>
        <div className="absolute bottom-1/3 right-1/4 w-72 h-72 bg-neon-pink-500/5 rounded-full filter blur-3xl animate-pulse"></div>
      </div>
      
      {/* Gradient overlay */}
      <div className="absolute inset-0 bg-gradient-to-br from-dark-bg to-dark-card opacity-90"></div>
      
      <div className="relative container mx-auto px-4 py-24">
        <div className="max-w-3xl mx-auto text-center">
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.5 }}
          >
            <h2 className="text-3xl md:text-5xl font-bold text-light-text mb-6">
              Ready to <span className="text-transparent bg-clip-text bg-gradient-to-r from-neon-purple-400 to-neon-pink-400">Explore</span> the Future?
            </h2>
            <p className="text-xl text-light-text/80 mb-10">
              Join thousands of students and researchers delving into the frontier of physical AI and humanoid robotics.
            </p>
          </motion.div>
          
          <motion.div 
            className="flex flex-wrap justify-center gap-4"
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.5, delay: 0.2 }}
          >
            <Button variant="primary" size="lg">
              Start Learning Now
            </Button>
            <Button variant="outline" size="lg">
              View Curriculum
            </Button>
          </motion.div>
          
          <motion.div
            className="mt-12 pt-8 border-t border-dark-card/50"
            initial={{ opacity: 0 }}
            animate={{ opacity: 1 }}
            transition={{ delay: 0.4, duration: 0.5 }}
          >
            <div className="flex flex-wrap justify-center items-center gap-8">
              <div className="text-light-text/70">
                <p className="text-2xl font-bold text-neon-purple-400">7</p>
                <p className="text-sm">Chapters</p>
              </div>
              <div className="text-light-text/70">
                <p className="text-2xl font-bold text-neon-pink-400">39</p>
                <p className="text-sm">Lessons</p>
              </div>
              <div className="text-light-text/70">
                <p className="text-2xl font-bold text-neon-purple-400">100+</p>
                <p className="text-sm">Exercises</p>
              </div>
              <div className="text-light-text/70">
                <p className="text-2xl font-bold text-neon-pink-400">24/7</p>
                <p className="text-sm">Support</p>
              </div>
            </div>
          </motion.div>
        </div>
      </div>
    </div>
  );
};

export default CTAFooterSection;