// Global types for the Physical AI & Humanoid Robotics platform

export interface Metric {
  value: string | number;
  label: string;
}

export interface Link {
  name: string;
  href: string;
}

export interface NavItem {
  name: string;
  href: string;
}

export interface Feature {
  title: string;
  description: string;
  icon?: string;
  iconColor?: 'primary' | 'secondary' | 'accent' | 'neon-purple' | 'neon-pink';
  badge?: string;
}

export interface SpectrumItem {
  type: 'assisted' | 'driven' | 'native';
  title: string;
  description: string;
  icon?: string;
}

export interface Book {
  title: string;
  author: string;
  description: string;
  coverImage?: string;
  chapters?: number;
  lessons?: number;
  exercises?: number;
}