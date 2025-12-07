// Motion utility functions and variants for animations
import { Variants } from 'framer-motion';

// Page entry animations
export const pageEntry: Variants = {
  hidden: { opacity: 0, y: 20 },
  visible: { 
    opacity: 1, 
    y: 0,
    transition: {
      duration: 0.5,
      staggerChildren: 0.1
    }
  }
};

// Stagger children animations
export const staggerChildren: Variants = {
  hidden: { opacity: 0 },
  visible: {
    opacity: 1,
    transition: {
      staggerChildren: 0.1
    }
  }
};

// Individual item animations
export const itemEntry: Variants = {
  hidden: { opacity: 0, y: 20 },
  visible: { 
    opacity: 1, 
    y: 0,
    transition: {
      duration: 0.5
    }
  }
};

// Hover animations
export const hoverLift: Variants = {
  rest: { 
    y: 0,
    transition: { duration: 0.2 }
  },
  hover: { 
    y: -10,
    transition: { 
      duration: 0.3,
      type: "spring",
      stiffness: 300,
      damping: 20
    }
  }
};

// Scale animations
export const hoverScale: Variants = {
  rest: { scale: 1 },
  hover: { scale: 1.05 }
};

// Fade in animations
export const fadeIn: Variants = {
  hidden: { opacity: 0 },
  visible: { 
    opacity: 1,
    transition: { duration: 0.5 }
  }
};

// Slide in from left
export const slideInLeft: Variants = {
  hidden: { opacity: 0, x: -50 },
  visible: { 
    opacity: 1, 
    x: 0,
    transition: { 
      duration: 0.5,
      ease: "easeOut"
    }
  }
};

// Slide in from right
export const slideInRight: Variants = {
  hidden: { opacity: 0, x: 50 },
  visible: { 
    opacity: 1, 
    x: 0,
    transition: { 
      duration: 0.5,
      ease: "easeOut"
    }
  }
};

// Slide in from top
export const slideInTop: Variants = {
  hidden: { opacity: 0, y: -50 },
  visible: { 
    opacity: 1, 
    y: 0,
    transition: { 
      duration: 0.5,
      ease: "easeOut"
    }
  }
};

// Slide in from bottom
export const slideInBottom: Variants = {
  hidden: { opacity: 0, y: 50 },
  visible: { 
    opacity: 1, 
    y: 0,
    transition: { 
      duration: 0.5,
      ease: "easeOut"
    }
  }
};

// Bounce animation
export const bounce: Variants = {
  rest: { y: 0 },
  hover: { 
    y: [0, -10, 0],
    transition: { 
      duration: 0.6,
      repeat: Infinity,
      repeatType: "loop" as const
    }
  }
};

// Pulse animation
export const pulse: Variants = {
  rest: { 
    boxShadow: "0 0 0 0 rgba(101, 101, 255, 0.4)" 
  },
  hover: { 
    boxShadow: "0 0 0 15px rgba(101, 101, 255, 0)",
    transition: {
      duration: 1,
      repeat: Infinity,
      repeatType: "loop" as const
    }
  }
};

// Gradient shimmer effect
export const gradientShimmer: Variants = {
  rest: {
    backgroundPosition: "0% 50%",
  },
  hover: {
    backgroundPosition: "100% 50%",
    transition: {
      duration: 1,
      repeat: Infinity,
      repeatType: "loop" as const,
      ease: "linear"
    }
  }
};

// Floating animation
export const float: Variants = {
  rest: { y: 0 },
  hover: { 
    y: [0, -10, 0],
    transition: { 
      duration: 3,
      repeat: Infinity,
      repeatType: "loop" as const,
      ease: "easeInOut"
    }
  }
};