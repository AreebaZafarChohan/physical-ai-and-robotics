# Backup and Recovery Procedures for Personalization Data

## Overview
This document describes the procedures to backup and recover personalization data, including user profiles, personalization rules, and cache data.

## Backup Procedures

### 1. Database Backup

#### Regular Automated Backups
- The Neon Postgres database automatically performs continuous backups
- Daily full backups are retained for 7 days
- Point-in-time recovery is available for the last 7 days

#### Manual Backup Process
1. Connect to your Neon database:
   ```bash
   psql "postgresql://username:password@ep-xxx.us-east-1.aws.neon.tech/dbname"
   ```

2. Create a backup of personalization-related tables:
   ```bash
   pg_dump --dbname="postgresql://username:password@ep-xxx.us-east-1.aws.neon.tech/dbname" \
           --table="user_profiles" \
           --table="personalization_rules" \
           --table="personalization_cache" \
           --table="content_markers" \
           --file="personalization-backup-$(date +%Y%m%d-%H%M%S).sql"
   ```

### 2. Configuration Backup
Backup any configuration files related to personalization:
```bash
# Copy configuration files
cp /path/to/backend/.env /backup/personalization-config-$(date +%Y%m%d).env
cp /path/to/backend/src/config.py /backup/personalization-config-$(date +%Y%m%d)/
```

### 3. Cache Data Considerations
- Redis cache data is ephemeral and does not need to be backed up
- Cache will be rebuilt automatically when the system recovers
- Document Redis configuration settings for recovery

## Recovery Procedures

### 1. Database Recovery

#### From Neon Automatic Backups
1. In the Neon console, navigate to your project
2. Go to the Branches tab
3. Select the "..." menu next to your branch
4. Choose "Restore branch"
5. Select your recovery point and enter a name for the new branch

#### From Manual SQL Backup
1. Create a new database or clean existing one:
   ```sql
   -- Connect to database
   DROP TABLE IF EXISTS user_profiles, personalization_rules, personalization_cache, content_markers CASCADE;
   ```

2. Restore the backup:
   ```bash
   psql -d "postgresql://username:password@ep-xxx.us-east-1.aws.neon.tech/dbname" \
        -f personalization-backup-YYYYMMDD-HHMMSS.sql
   ```

### 2. Application Recovery
1. Deploy the application code to the recovery environment
2. Update environment variables with correct database connection details
3. Run database migrations if needed:
   ```bash
   python -m alembic upgrade head
   ```

### 3. Cache Recovery
1. Restart Redis service
2. Redis will start empty, and cache will be rebuilt automatically
3. Monitor cache hit rates as they return to normal

## Recovery Scenarios

### Complete System Failure
1. Provision new infrastructure
2. Deploy application code
3. Restore database from latest backup
4. Update configuration
5. Test personalization functionality
6. Gradually route traffic to recovered system

### Data Corruption
1. Identify time of corruption
2. Restore to a point before corruption occurred
3. Apply any legitimate changes made after the backup point
4. Validate personalization functionality
5. Resume normal operations

## Testing Procedures

### Backup Validation
- Regularly verify backup files can be restored to a test environment
- Validate that all personalization tables are correctly backed up
- Check that foreign key relationships are preserved

### Recovery Testing
- Conduct recovery tests quarterly
- Document recovery time and any issues encountered
- Update procedures based on test results

## Responsibilities

- **System Administrator**: Execute backup and recovery procedures
- **Database Administrator**: Monitor database backup health
- **Development Team**: Provide support for application-level recovery
- **Security Team**: Verify backup security and access controls

## Contact Information

For backup and recovery assistance:
- System Administrator: [admin-contact]
- Database Administrator: [dba-contact]
- Development Team: [dev-team-contact]