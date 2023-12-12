pub fn initPartitionByName(s: &String) -> u8
{
    let strPtr = s.as_ptr() as *const i8;
    unsafe
    {
        let retVal = crate::FLASH_INIT_PARTITION(strPtr);
        retVal
    }
}

pub fn erasePartitionByName(s: &String) -> u8
{
    let strPtr = s.as_ptr() as *const i8;
    unsafe
    {
        let retVal = crate::FLASH_ERASE_PARTITION(strPtr);
        retVal
    }
}

/* not currently being used
pub fn getPartitionInfo(s: &String) -> crate::PARTITION_INFO_t
{
    let strPtr = s.as_ptr() as *const i8;
    unsafe
    {
        let retVal = crate::FLASH_GET_PARTITION_INFO(strPtr);
        retVal
    }
}
*/

pub fn askIfKeyExists(partition: &String, namespace: &String, blob: &String) -> usize
{
    let partPtr = partition.as_ptr() as *const i8;
    let nmPtr = namespace.as_ptr() as *const i8;
    let blobPtr = blob.as_ptr() as *const i8;
    unsafe
    {
        let retVal = crate::FLASH_DOES_KEY_EXIST(partPtr, nmPtr, blobPtr);
        retVal
    }
}

pub fn writeBlobToKey(partition: &String, namespace: &String, blob: &String, data: Vec<u8>, size: usize) -> u8
{
    let partPtr = partition.as_ptr() as *const i8;
    let nmPtr = namespace.as_ptr() as *const i8;
    let blobPtr = blob.as_ptr() as *const i8;
    let datPtr = data.as_ptr() as *const u8;
    unsafe
    {
        let retVal = crate::FLASH_WRITE_TO_BLOB(partPtr, nmPtr, blobPtr, datPtr, size);
        retVal
    }
}

pub fn readBlobFromKey(partition: &String, namespace: &String, blob: &String, size: usize) -> Vec<u8>
{
    let partPtr = partition.as_ptr() as *const i8;
    let nmPtr = namespace.as_ptr() as *const i8;
    let blobPtr = blob.as_ptr() as *const i8;
    unsafe
    {
        let blobData = crate::FLASH_READ_FROM_BLOB(partPtr, nmPtr, blobPtr, size);
        let blobSlice = Vec::from_raw_parts(blobData, size, size);
        blobSlice
    }
}
