import { 
  getCurrentLocale, 
  useCurrentLocale, 
  isRTL, 
  getDirection, 
  getLocaleBaseUrl, 
  translatePath, 
  isValidLocale, 
  getDirectionClassName 
} from '../locale';

// Mock the router
jest.mock('@docusaurus/router', () => ({
  useLocation: jest.fn(() => ({ pathname: '/en/docs/intro' })),
}));

describe('Locale Utilities', () => {
  beforeEach(() => {
    // Reset any mocks
    jest.clearAllMocks();
  });

  describe('getCurrentLocale', () => {
    test('should return "en" as default when no locale in path', () => {
      // Mock window.location for the test
      Object.defineProperty(window, 'location', {
        value: {
          pathname: '/docs/intro',
        },
        writable: true,
      });

      const locale = getCurrentLocale();
      expect(locale).toBe('en');
    });

    test('should return locale from path', () => {
      Object.defineProperty(window, 'location', {
        value: {
          pathname: '/ur/docs/intro',
        },
        writable: true,
      });

      const locale = getCurrentLocale();
      expect(locale).toBe('ur');
    });

    test('should return "en" for unsupported locale in path', () => {
      Object.defineProperty(window, 'location', {
        value: {
          pathname: '/fr/docs/intro',
        },
        writable: true,
      });

      const locale = getCurrentLocale();
      expect(locale).toBe('en');
    });
  });

  describe('isRTL', () => {
    test('should return true for RTL languages', () => {
      expect(isRTL('ur')).toBe(true);
      expect(isRTL('ar')).toBe(true);
    });

    test('should return false for LTR languages', () => {
      expect(isRTL('en')).toBe(false);
    });

    test('should return false for default locale', () => {
      expect(isRTL()).toBe(false); // Default is 'en'
    });
  });

  describe('getDirection', () => {
    test('should return "rtl" for RTL languages', () => {
      expect(getDirection('ur')).toBe('rtl');
      expect(getDirection('ar')).toBe('rtl');
    });

    test('should return "ltr" for LTR languages', () => {
      expect(getDirection('en')).toBe('ltr');
    });
  });

  describe('getLocaleBaseUrl', () => {
    test('should return "/" for English (default) locale', () => {
      expect(getLocaleBaseUrl('en')).toBe('/');
    });

    test('should return locale-specific path for other locales', () => {
      expect(getLocaleBaseUrl('ur')).toBe('/ur/');
      expect(getLocaleBaseUrl('ar')).toBe('/ar/');
    });
  });

  describe('translatePath', () => {
    test('should translate path to different locale', () => {
      const result = translatePath('/en/docs/intro', 'ur');
      expect(result).toBe('/ur/docs/intro');
    });

    test('should handle path without locale prefix', () => {
      const result = translatePath('/docs/intro', 'ur');
      expect(result).toBe('/ur/docs/intro');
    });

    test('should translate to English without prefix', () => {
      const result = translatePath('/ur/docs/intro', 'en');
      expect(result).toBe('/docs/intro');
    });
  });

  describe('isValidLocale', () => {
    test('should return true for valid locales', () => {
      expect(isValidLocale('en')).toBe(true);
      expect(isValidLocale('ur')).toBe(true);
      expect(isValidLocale('ar')).toBe(true);
    });

    test('should return false for invalid locales', () => {
      expect(isValidLocale('fr')).toBe(false);
      expect(isValidLocale('invalid')).toBe(false);
    });
  });

  describe('getDirectionClassName', () => {
    test('should return "rtl" class for RTL languages', () => {
      expect(getDirectionClassName('ur')).toBe('rtl');
      expect(getDirectionClassName('ar')).toBe('rtl');
    });

    test('should return "ltr" class for LTR languages', () => {
      expect(getDirectionClassName('en')).toBe('ltr');
    });
  });
});