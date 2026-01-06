from typing import Optional, List, Dict, Any
from pydantic import BaseModel
from datetime import datetime
import asyncio
import os
from pathlib import Path
import json

class TranslationJob(BaseModel):
    id: str
    sourceFilePath: str
    targetFilePath: str
    status: str  # 'pending', 'in_progress', 'completed', 'failed', 'review_required'
    sourceLanguage: str
    targetLanguage: str
    aiModelUsed: str
    timestamp: datetime
    completionTime: Optional[datetime] = None
    notes: Optional[str] = None

class TranslationRequest(BaseModel):
    sourceFiles: List[str]
    targetLanguage: str
    sourceLanguage: str
    forceUpdate: bool = False

class TranslationStatusResponse(BaseModel):
    jobId: str
    status: str
    progress: int
    files: List[Dict[str, str]]
    completionTime: Optional[datetime] = None
    message: Optional[str] = None

class AvailableTranslationsResponse(BaseModel):
    translations: List[Dict[str, Any]]

class UpdateTranslationRequest(BaseModel):
    filePath: str
    content: str
    reviewStatus: str  # 'approved', 'needs_review'

class LocaleInfo(BaseModel):
    code: str
    name: str
    dir: str  # "ltr", "rtl"
    path: str  # URL path prefix

class LocalesResponse(BaseModel):
    locales: List[LocaleInfo]

class UserPreferenceRequest(BaseModel):
    locale: str
    persist: bool = True

class UserPreferenceResponse(BaseModel):
    status: str
    message: str
    redirectUrl: Optional[str] = None

class TranslationService:
    def __init__(self):
        self.translation_jobs = {}
        self.translation_queue = asyncio.Queue()
    
    async def trigger_translation(self, request: TranslationRequest) -> Dict[str, Any]:
        """Initiates the translation process for English to Urdu"""
        job_id = f"job_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}"
        
        # Create a new translation job
        job = TranslationJob(
            id=job_id,
            sourceFilePath=request.sourceFiles[0] if request.sourceFiles else "",
            targetFilePath="",  # Will be set during processing
            status="initiated",
            sourceLanguage=request.sourceLanguage,
            targetLanguage=request.targetLanguage,
            aiModelUsed="Claude-3",
            timestamp=datetime.now()
        )
        
        self.translation_jobs[job_id] = job
        
        # Add to processing queue
        for file_path in request.sourceFiles:
            await self.translation_queue.put({
                "job_id": job_id,
                "source_file": file_path,
                "target_language": request.targetLanguage,
                "source_language": request.sourceLanguage,
                "force_update": request.forceUpdate
            })
        
        # Start processing in background
        asyncio.create_task(self.process_translation_queue())
        
        return {
            "jobId": job_id,
            "status": "initiated",
            "filesProcessed": len(request.sourceFiles),
            "message": f"Translation job {job_id} initiated for {len(request.sourceFiles)} files"
        }
    
    async def get_translation_status(self, job_id: str) -> TranslationStatusResponse:
        """Gets the status of a specific translation job"""
        if job_id not in self.translation_jobs:
            raise ValueError(f"Job {job_id} not found")
        
        job = self.translation_jobs[job_id]
        
        # In a real implementation, we would track actual progress
        # For now, return a mock response
        return TranslationStatusResponse(
            jobId=job.id,
            status=job.status,
            progress=50,  # Placeholder
            files=[{
                "sourcePath": job.sourceFilePath,
                "targetPath": job.targetFilePath,
                "status": job.status
            }],
            completionTime=job.completionTime,
            message=f"Job {job_id} is {job.status}"
        )
    
    async def get_bulk_translation_status(self, limit: int = 10, status: Optional[str] = None) -> Dict[str, List[Dict[str, Any]]]:
        """Gets the status of all recent translation jobs"""
        jobs = []
        for job in list(self.translation_jobs.values())[:limit]:
            if status is None or job.status == status:
                jobs.append({
                    "jobId": job.id,
                    "status": job.status,
                    "sourceLanguage": job.sourceLanguage,
                    "targetLanguage": job.targetLanguage,
                    "timestamp": job.timestamp,
                    "completionTime": job.completionTime
                })
        
        return {"jobs": jobs}
    
    async def get_available_translations(self, file_path: Optional[str] = None) -> AvailableTranslationsResponse:
        """Lists all available translations for content"""
        # This would check the docs directory for available translations
        translations = []
        
        # For now, return a mock response
        if file_path:
            translations.append({
                "filePath": file_path,
                "availableLocales": ["en", "ur"],  # Example
                "lastUpdated": datetime.now()
            })
        else:
            # Return all available translations
            translations.append({
                "filePath": "intro.mdx",
                "availableLocales": ["en", "ur"],
                "lastUpdated": datetime.now()
            })
        
        return AvailableTranslationsResponse(translations=translations)
    
    async def update_translation(self, request: UpdateTranslationRequest) -> Dict[str, str]:
        """Updates a specific translation file"""
        # In a real implementation, this would update the translation file
        # For now, return a mock response
        return {
            "status": "success",
            "message": f"Translation for {request.filePath} updated successfully",
            "filePath": request.filePath
        }
    
    async def get_available_locales(self) -> LocalesResponse:
        """Gets all supported locales for the site"""
        locales = [
            LocaleInfo(code="en", name="English", dir="ltr", path="/en/"),
            LocaleInfo(code="ur", name="اردو", dir="rtl", path="/ur/"),
            LocaleInfo(code="ar", name="العربية", dir="rtl", path="/ar/")
        ]
        return LocalesResponse(locales=locales)
    
    async def set_user_preference(self, request: UserPreferenceRequest) -> UserPreferenceResponse:
        """Sets the user's language preference"""
        # In a real implementation, this would store the user preference
        # For now, return a mock response
        redirect_url = f"/{request.locale}/" if request.persist else None
        return UserPreferenceResponse(
            status="success",
            message=f"User preference set to {request.locale}",
            redirectUrl=redirect_url
        )
    
    async def process_translation_queue(self):
        """Process translation jobs in the queue"""
        while not self.translation_queue.empty():
            job_data = await self.translation_queue.get()
            job_id = job_data["job_id"]
            
            # Update job status
            if job_id in self.translation_jobs:
                self.translation_jobs[job_id].status = "in_progress"
            
            # Simulate translation process
            await asyncio.sleep(2)  # Simulate API call time
            
            # Update job status to completed
            if job_id in self.translation_jobs:
                self.translation_jobs[job_id].status = "completed"
                self.translation_jobs[job_id].completionTime = datetime.now()
                self.translation_jobs[job_id].targetFilePath = f"docs/{job_data['target_language']}/{job_data['source_file'].split('/')[-1]}"