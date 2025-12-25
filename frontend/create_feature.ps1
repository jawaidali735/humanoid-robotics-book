# PowerShell script to create new feature
# This will be executed through bash

param(
    [string]$FeatureDescription,
    [int]$Number = 1,
    [string]$ShortName = "text-embedding-retrieval"
)

# Output JSON for the feature creation
$featureInfo = @{
    BRANCH_NAME = "$Number-$ShortName"
    SPEC_FILE = "specs/$Number-$ShortName/spec.md"
    FEATURE_DIR = "specs/$Number-$ShortName"
}

Write-Output ($featureInfo | ConvertTo-Json)