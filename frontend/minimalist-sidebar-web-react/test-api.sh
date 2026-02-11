#!/bin/bash

# Comprehensive API Testing Script for Tasks Management System
# Tests all backend endpoints and verifies functionality

set -e

API_BASE="http://localhost:3000/api/trpc"
RESULTS_FILE="api-test-results.txt"

echo "==================================" | tee $RESULTS_FILE
echo "API Testing Started: $(date)" | tee -a $RESULTS_FILE
echo "==================================" | tee -a $RESULTS_FILE
echo "" | tee -a $RESULTS_FILE

# Test counter
TOTAL_TESTS=0
PASSED_TESTS=0
FAILED_TESTS=0

# Test function
test_endpoint() {
    local name=$1
    local method=$2
    local endpoint=$3
    local data=$4
    
    TOTAL_TESTS=$((TOTAL_TESTS + 1))
    echo "Testing: $name" | tee -a $RESULTS_FILE
    
    if [ "$method" = "GET" ]; then
        response=$(curl -s "$API_BASE/$endpoint")
    else
        response=$(curl -s -X POST -H "Content-Type: application/json" -d "$data" "$API_BASE/$endpoint")
    fi
    
    if echo "$response" | jq -e '.result' > /dev/null 2>&1; then
        echo "  ✅ PASS" | tee -a $RESULTS_FILE
        PASSED_TESTS=$((PASSED_TESTS + 1))
        return 0
    else
        echo "  ❌ FAIL" | tee -a $RESULTS_FILE
        echo "  Response: $response" | tee -a $RESULTS_FILE
        FAILED_TESTS=$((FAILED_TESTS + 1))
        return 1
    fi
}

echo "1. TASKS CRUD ENDPOINTS" | tee -a $RESULTS_FILE
echo "========================" | tee -a $RESULTS_FILE

# Test tasks.list
test_endpoint "tasks.list" "GET" "tasks.list"

# Test tasks.create
CREATE_DATA='{"json":{"taskId":"TASK-1","userId":1,"title":"Test Task","description":"This is a test task","status":"todo","priority":"medium","tags":[],"progress":0,"starred":false,"hasInProgressAttempt":false,"lastAttemptFailed":false,"isRecurring":false}}'
test_endpoint "tasks.create" "POST" "tasks.create" "$CREATE_DATA"

# Get the created task ID
TASK_ID=$(mysql -uroot -ppassword ripple_db -se "SELECT id FROM tasks LIMIT 1;" 2>/dev/null)

if [ -n "$TASK_ID" ]; then
    echo "  Task ID: $TASK_ID" | tee -a $RESULTS_FILE
    
    # Test tasks.getById
    response=$(curl -s "$API_BASE/tasks.getById?input=%7B%22json%22%3A%7B%22id%22%3A$TASK_ID%7D%7D")
    if echo "$response" | jq -e '.result' > /dev/null 2>&1; then
        echo "Testing: tasks.getById" | tee -a $RESULTS_FILE
        echo "  ✅ PASS" | tee -a $RESULTS_FILE
        PASSED_TESTS=$((PASSED_TESTS + 1))
    else
        echo "Testing: tasks.getById" | tee -a $RESULTS_FILE
        echo "  ❌ FAIL" | tee -a $RESULTS_FILE
        FAILED_TESTS=$((FAILED_TESTS + 1))
    fi
    TOTAL_TESTS=$((TOTAL_TESTS + 1))
    
    # Test tasks.update
    UPDATE_DATA="{\"json\":{\"id\":$TASK_ID,\"title\":\"Updated Task\"}}"
    test_endpoint "tasks.update" "POST" "tasks.update" "$UPDATE_DATA"
    
    # Test tasks.updateStatus
    STATUS_DATA="{\"json\":{\"id\":$TASK_ID,\"status\":\"in_progress\"}}"
    test_endpoint "tasks.updateStatus" "POST" "tasks.updateStatus" "$STATUS_DATA"
    
    # Test tasks.star
    STAR_DATA="{\"json\":{\"id\":$TASK_ID,\"starred\":true}}"
    test_endpoint "tasks.star" "POST" "tasks.star" "$STAR_DATA"
    
    # Test tasks.updateProgress
    PROGRESS_DATA="{\"json\":{\"id\":$TASK_ID,\"progress\":50}}"
    test_endpoint "tasks.updateProgress" "POST" "tasks.updateProgress" "$PROGRESS_DATA"
fi

echo "" | tee -a $RESULTS_FILE
echo "2. COMMENTS ENDPOINTS" | tee -a $RESULTS_FILE
echo "=====================" | tee -a $RESULTS_FILE

if [ -n "$TASK_ID" ]; then
    # Test comments.create
    COMMENT_DATA="{\"json\":{\"taskId\":$TASK_ID,\"author\":\"Test User\",\"content\":\"This is a test comment\"}}"
    test_endpoint "comments.create" "POST" "tasks.comments.create" "$COMMENT_DATA"
    
    # Test comments.list
    test_endpoint "comments.list" "POST" "tasks.comments.list" "{\"json\":{\"taskId\":$TASK_ID}}"
fi

echo "" | tee -a $RESULTS_FILE
echo "3. ACTIVITIES ENDPOINTS" | tee -a $RESULTS_FILE
echo "=======================" | tee -a $RESULTS_FILE

if [ -n "$TASK_ID" ]; then
    # Test activities.create
    ACTIVITY_DATA="{\"json\":{\"taskId\":$TASK_ID,\"type\":\"created\",\"user\":\"Test User\",\"description\":\"Task created\"}}"
    test_endpoint "activities.create" "POST" "tasks.activities.create" "$ACTIVITY_DATA"
    
    # Test activities.list
    test_endpoint "activities.list" "POST" "tasks.activities.list" "{\"json\":{\"taskId\":$TASK_ID}}"
fi

echo "" | tee -a $RESULTS_FILE
echo "4. TIME TRACKING ENDPOINTS" | tee -a $RESULTS_FILE
echo "==========================" | tee -a $RESULTS_FILE

if [ -n "$TASK_ID" ]; then
    # Test timeEntries.create
    TIME_DATA="{\"json\":{\"taskId\":$TASK_ID,\"user\":\"Test User\",\"duration\":3600}}"
    test_endpoint "timeEntries.create" "POST" "tasks.timeEntries.create" "$TIME_DATA"
    
    # Test timeEntries.list
    test_endpoint "timeEntries.list" "POST" "tasks.timeEntries.list" "{\"json\":{\"taskId\":$TASK_ID}}"
fi

echo "" | tee -a $RESULTS_FILE
echo "5. AI FEATURES ENDPOINTS" | tee -a $RESULTS_FILE
echo "========================" | tee -a $RESULTS_FILE

# Test AI.suggestTasks
test_endpoint "ai.suggestTasks" "POST" "tasks.ai.suggestTasks" "{\"json\":{}}"

if [ -n "$TASK_ID" ]; then
    # Test AI.generateSubtasks
    test_endpoint "ai.generateSubtasks" "POST" "tasks.ai.generateSubtasks" "{\"json\":{\"taskId\":$TASK_ID}}"
    
    # Test AI.predictPriority
    test_endpoint "ai.predictPriority" "POST" "tasks.ai.predictPriority" "{\"json\":{\"taskId\":$TASK_ID}}"
    
    # Test AI.estimateDeadline
    test_endpoint "ai.estimateDeadline" "POST" "tasks.ai.estimateDeadline" "{\"json\":{\"taskId\":$TASK_ID}}"
    
    # Test AI.analyzeTask
    test_endpoint "ai.analyzeTask" "POST" "tasks.ai.analyzeTask" "{\"json\":{\"taskId\":$TASK_ID}}"
fi

# Test AI.getInsights
test_endpoint "ai.getInsights" "POST" "tasks.ai.getInsights" "{\"json\":{}}"

echo "" | tee -a $RESULTS_FILE
echo "6. RECURRING TASKS ENDPOINTS" | tee -a $RESULTS_FILE
echo "============================" | tee -a $RESULTS_FILE

# Test recurring.create
RECURRING_DATA='{"json":{"taskId":"TASK-REC-1","userId":1,"title":"Recurring Task","status":"todo","priority":"medium","tags":[],"progress":0,"starred":false,"hasInProgressAttempt":false,"lastAttemptFailed":false,"isRecurring":true,"recurrencePattern":"daily"}}'
test_endpoint "recurring.create" "POST" "tasks.recurring.create" "$RECURRING_DATA"

# Test recurring.list
test_endpoint "recurring.list" "GET" "tasks.recurring.list"

echo "" | tee -a $RESULTS_FILE
echo "==================================" | tee -a $RESULTS_FILE
echo "TEST SUMMARY" | tee -a $RESULTS_FILE
echo "==================================" | tee -a $RESULTS_FILE
echo "Total Tests: $TOTAL_TESTS" | tee -a $RESULTS_FILE
echo "Passed: $PASSED_TESTS" | tee -a $RESULTS_FILE
echo "Failed: $FAILED_TESTS" | tee -a $RESULTS_FILE
echo "Success Rate: $(echo "scale=2; $PASSED_TESTS * 100 / $TOTAL_TESTS" | bc)%" | tee -a $RESULTS_FILE
echo "==================================" | tee -a $RESULTS_FILE
echo "Testing Completed: $(date)" | tee -a $RESULTS_FILE
echo "==================================" | tee -a $RESULTS_FILE

# Exit with error if any tests failed
if [ $FAILED_TESTS -gt 0 ]; then
    exit 1
fi

exit 0
