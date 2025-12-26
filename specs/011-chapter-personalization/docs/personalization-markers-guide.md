# Personalization Markers Guide for Content Authors

This guide explains how to use personalization markers in your content to provide tailored experiences for different types of users based on their software, hardware background, and experience level.

## Marker Syntax

Personalization markers follow this format:

```
{{personalize:marker_type}}Your personalized content here{{personalize:end}}
```

## Available Marker Types

### Experience Level Markers
- `{{personalize:beginner}}...{{personalize:end}}` - Content for beginners
- `{{personalize:intermediate}}...{{personalize:end}}` - Content for intermediate users
- `{{personalize:advanced}}...{{personalize:end}}` - Content for advanced users

### Software Background Markers
- `{{personalize:python}}...{{personalize:end}}` - Content for Python developers
- `{{personalize:javascript}}...{{personalize:end}}` - Content for JavaScript developers
- `{{personalize:react}}...{{personalize:end}}` - Content for React developers
- `{{personalize:nodejs}}...{{personalize:end}}` - Content for Node.js developers

### Hardware Background Markers
- `{{personalize:raspberry-pi}}...{{personalize:end}}` - Content for Raspberry Pi users
- `{{personalize:arduino}}...{{personalize:end}}` - Content for Arduino users
- `{{personalize:esp32}}...{{personalize:end}}` - Content for ESP32 users

### Combined Markers
You can create markers that target specific combinations:
- `{{personalize:python-raspberry-pi}}...{{personalize:end}}` - Content targeting both Python and Raspberry Pi users

## Examples

### Basic Example
```
This content is shown to all users.

{{personalize:beginner}}
This content is specifically for beginners. It explains concepts in simpler terms.
{{personalize:end}}

{{personalize:python}}
This content is for Python developers and may include Python-specific examples.
{{personalize:end}}
```

### Multiple Markers Example
```
{{personalize:intermediate}}
This section provides more detailed information for intermediate users.
{{personalize:end}}

{{personalize:raspberry-pi}}
This content includes Raspberry Pi-specific instructions.
{{personalize:end}}

{{personalize:python}}
Here are some Python code examples:
```python
print("Hello from Raspberry Pi!")
```
{{personalize:end}}
```

## Best Practices

1. **Keep It Simple**: Use clear and descriptive marker types that match what users would select in their profiles.

2. **Fallback Content**: Provide general content that applies to all users outside of personalized markers.

3. **Consistent Formatting**: Keep the look and feel consistent across personalized sections.

4. **Test Thoroughly**: Test your content with different user profiles to ensure the right content appears for the right audience.

5. **Avoid Duplicates**: Don't put the same content in multiple markers as this can lead to confusion.

## Security Note

All content between markers is sanitized for security, so avoid using JavaScript or other executable code within markers.

## Troubleshooting

- If your personalized content doesn't appear, check that the marker types match exactly with what users specify in their profiles
- If content appears for everyone, make sure you've properly closed all markers with `{{personalize:end}}`
- If you're getting formatting issues, ensure that the HTML structure inside markers is properly balanced