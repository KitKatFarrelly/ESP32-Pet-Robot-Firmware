use std::slice;

pub fn initPartitionByName(s: &str) -> u8
{
    let strPtr = s.as_ptr() as *const i8;
    unsafe
    {
        let retVal = crate::FLASH_INIT_PARTITION(strPtr);
        retVal
    }
}

pub fn erasePartitionByName(s: &str) -> u8
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

pub fn askIfKeyExists(partition: &str, namespace: &str, blob: &str) -> usize
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

pub fn writeBlobToKey(partition: &str, namespace: &str, blob: &str, data: Vec<u8>, size: usize) -> u8
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

pub fn readBlobFromKey(partition: &str, namespace: &str, blob: &str, size: usize) -> Vec<u8>
{
    let partPtr = partition.as_ptr() as *const i8;
    let nmPtr = namespace.as_ptr() as *const i8;
    let blobPtr = blob.as_ptr() as *const i8;
    unsafe
    {
        let blobData = crate::FLASH_READ_FROM_BLOB(partPtr, nmPtr, blobPtr, size) as *const u8;
        let blobVec = slice::from_raw_parts(blobData, size).to_vec();
        blobVec
    }
}

#[cfg(test)]
mod tests
{
    use super::*;
    
    static test_partition: &str = "factory\0";
    static test_namespace: &str = "rustns\0";
    static test_blob_1: &str = "rust_blob_1";
    static test_blob_2: &str = "rust_blob_2";
    
    #[test]
    fn test_init_partition()
    {
        assert_eq!(0, initPartitionByName(test_partition));
    }

    #[test]
    fn test_erase_partition()
    {
        assert_eq!(0, erasePartitionByName(test_partition));
    }

    #[test]
    fn test_read_key_does_not_exist()
    {
        let key_size = askIfKeyExists(test_partition, test_namespace, test_blob_1);
        assert_eq!(0, key_size);
        let output_vec = readBlobFromKey(test_partition, test_namespace, test_blob_1, key_size);

        let vec_iter = output_vec.iter();
        assert_eq!(0, output_vec.len());
    }

    #[test]
    fn test_read_write_key_does_exist()
    {
        let input_list = vec![1,4,6,0xF,0xE,2,5,0];
        let input_copy = input_list.clone();

        let input_len = input_list.len();

        assert_eq!(0, writeBlobToKey(test_partition, test_namespace, test_blob_1, input_list, input_len));
        
        let key_size = askIfKeyExists(test_partition, test_namespace, test_blob_1);
        assert_eq!(input_len, key_size);
        let output_vec = readBlobFromKey(test_partition, test_namespace, test_blob_1, key_size);

        assert_eq!(input_len, output_vec.len());
        let vec_iter = output_vec.iter();
        assert!(input_copy == output_vec);
    }

    #[test]
    fn test_read_write_multiple_keys()
    {

    }
}