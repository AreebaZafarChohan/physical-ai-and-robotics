import os
from pathlib import Path
from typing import Optional
from fastapi import HTTPException, status


class ContentService:
    """
    Service class for content-related operations
    """
    
    def __init__(self, content_base_path: str = "content"):
        """
        Initialize the content service with the base path for content files
        """
        self.content_base_path = Path(content_base_path)
        
        # Verify the content directory exists
        if not self.content_base_path.exists():
            raise ValueError(f"Content directory does not exist: {self.content_base_path}")
    
    async def get_content_by_path(self, content_path: str) -> str:
        """
        Get content by its path in the content directory structure
        """
        # Sanitize the path to prevent directory traversal attacks
        # Remove leading slash and normalize the path
        normalized_path = os.path.normpath(content_path.lstrip('/'))
        
        # Construct the full file path
        full_path = self.content_base_path / normalized_path
        
        # Add debug logging
        print(f"Debug: Attempting to access full_path: {full_path.resolve()}")
        print(f"Debug: Does full_path exist and is it a file? {full_path.is_file()}")

        
        # Verify that the path is within the content directory (security check)
        try:
            full_path.resolve().relative_to(self.content_base_path.resolve())
        except ValueError:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Invalid content path"
            )
            
        if full_path.suffix == "":
            full_path = full_path.with_suffix(".md")    
        
        # Ensure the file has an appropriate extension
        if full_path.suffix.lower() not in ['.md', '.mdx', '.txt', '.html']:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Invalid file type"
            )
        
        # Check if the file exists
        if not full_path.is_file():
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Content not found: {content_path}"
            )
        
        # Read and return the content
        try:
            with open(full_path, 'r', encoding='utf-8') as file:
                return file.read()
        except Exception as e:
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail=f"Error reading content: {str(e)}"
            )
    
    async def get_content_markers(self, content: str) -> list:
        """
        Extract personalization markers from content
        """
        # Looking for markers in the format: {{personalize:marker_type}}...{{personalize:end}}
        import re
        
        # Regular expression to match personalization markers
        pattern = r'\{\{personalize:([^}]+)\}\}([\s\S]*?)\{\{personalize:end\}\}'
        matches = re.findall(pattern, content)
        
        markers = []
        for marker_type, marker_content in matches:
            markers.append({
                'type': marker_type,
                'content': marker_content.strip()
            })
        
        return markers
    
    async def get_all_content_paths(self, extension: str = ".md") -> list:
        """
        Get all content paths with a specific extension
        """
        paths = []
        for file_path in self.content_base_path.rglob(f'*{extension}'):
            # Convert to relative path from content base
            relative_path = file_path.relative_to(self.content_base_path)
            paths.append(str(relative_path))
        
        return paths