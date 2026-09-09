use std::io::Result;
use std::path::PathBuf;

fn main() -> Result<()> {
    let out_dir = PathBuf::from("src/proto");

    // Build station protobufs
    prost_build::Config::new()
        .out_dir(&out_dir)
        .bytes(["."])
        .compile_protos(
            &["../../../protobufs/drivers/st3215/st3215.proto"],
            &["../../../protobufs/drivers/st3215"],
        )?;

    // Rerun if station protobufs change
    println!("cargo:rerun-if-changed=../../../protobufs/drivers/st3215/st3215.proto");

    Ok(())
}
