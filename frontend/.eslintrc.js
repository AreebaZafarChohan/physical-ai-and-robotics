module.exports = {
  root: true,
  extends: [
    '@docusaurus',
    'plugin:mdx/recommended',
  ],
  parserOptions: {
    ecmaVersion: 'latest',
    sourceType: 'module',
  },
  plugins: [
    'mdx',
  ],
  settings: {
    'mdx/code-blocks': true,
  },
  rules: {
    'no-unused-vars': 'warn',
    'no-console': 'warn',
    'prefer-const': 'error',
  },
  overrides: [
    {
      files: ['*.mdx', '*.md'],
      rules: {
        'no-undef': 'off',
      },
    },
  ],
};